package us.ihmc.convexOptimization;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HashSet;

import javax.xml.bind.DatatypeConverter;

import com.sun.jna.Native;

public class NativeLibraryLoader {

	public final static String LIBRARY_LOCATION = new File(
			System.getProperty("user.home"), ".ihmc" + File.separator + "lib")
			.getAbsolutePath();

	private NativeLibraryLoader() {
		// Disallow construction
	}

	/* these are tailored with CMake default dynamic library names */
	private static String get_OSX_MAC(String baseName) {
		return "lib" + baseName + "_rel.dylib";
	}

	private static String get_LINUX_64(String baseName) {
		return "lib" + baseName + "_rel.so";
	}

	private static String get_WINDOWS_64(String baseName) {
		return baseName + "_rel.dll";
	}

	private static final HashSet<String> loadedLibraries = new HashSet<String>();

	private static String createPackagePrefix(String packageName) {
		packageName = packageName.trim().replace('.', '/');
		if (packageName.length() > 0) {
			packageName = packageName + '/';
		}
		return packageName;
	}

	public static String getOSDependentName(String baseName) {

		if (isX86_64()) {
			if (isLinux()) {
				return get_LINUX_64(baseName);
			} else if (isMac()) {
				return get_OSX_MAC(baseName);
			} else if (isWindows()) {
				return get_WINDOWS_64(baseName);
			}
		}

		throw new RuntimeException(
				System.getProperty("os.name")
						+ "/"
						+ System.getProperty("os.arch")
						+ " unsupported. Only 64bit Linux/Mac/Windows supported for now.");
	}

	public synchronized static void loadJNALibraryFromClassPath(
			String packageName, String library, Class<?> interfaceClassJNA) {
		if (loadedLibraries.contains(library)) {
			return;
		}
	      try{
			Native.register(interfaceClassJNA,
					extractLibraryFromClassPath(packageName, library));
	      }
	      catch(NoSuchMethodError e)
	      {
	    	  throw new RuntimeException("NativeLibraryLoader: JNA too old");
	      }
	      loadedLibraries.add(library);
	}

	public synchronized static void loadJNILibraryFromClassPath(
			String packageName, String library) {
		if (loadedLibraries.contains(library)) {
			return;
		}
		System.load(extractLibraryFromClassPath(packageName, library));
	}

	private synchronized static String extractLibraryFromClassPath(
			String packageName, String library) {
		String packagePrefix = createPackagePrefix(packageName);
		File directory = new File(LIBRARY_LOCATION + "/" + packagePrefix);
		if (!directory.exists()) {
			directory.mkdirs();
		}
			String fileName = packagePrefix + library;
			InputStream stream = NativeLibraryLoader.class.getClassLoader()
					.getResourceAsStream(fileName);
			if (stream == null)
				throw new RuntimeException("Library not found:" + fileName);
			File lib=writeStreamToFile(stream, directory);

			try {
				stream.close();
			} catch (IOException e) {
			}

		loadedLibraries.add(library);
		return lib.getAbsolutePath();
	}

	public static File writeStreamToFile(InputStream stream, File directory) {
		try {
		   MessageDigest messageDigest = MessageDigest.getInstance("SHA-1");
		   DigestInputStream digestStream = new DigestInputStream(stream, messageDigest);
		   File file = File.createTempFile(".NativeLibraryLoader", null, directory);
		   
			FileOutputStream out = new FileOutputStream(file);
			byte[] buf = new byte[1024];
			int len;
			while ((len = digestStream.read(buf)) > 0) {
				out.write(buf, 0, len);
			}

			out.close();
			
			String filename = System.mapLibraryName(DatatypeConverter.printHexBinary(messageDigest.digest()));
			File target = new File(directory, filename);
			if(!target.exists())
			{
			   file.renameTo(target);
			}
			else
			{
			   file.delete();
			}
			return target;

		} catch (IOException|NoSuchAlgorithmException e) {
			throw new RuntimeException(e);
		}
		

	}

	public static boolean isWindows() {
		return System.getProperty("os.name").startsWith("Windows");
	}

	public static boolean isLinux() {
		return System.getProperty("os.name").equals("Linux");
	}

	public static boolean isMac() {
		return System.getProperty("os.name").equals("MacOSX")
				|| System.getProperty("os.name").equals("Mac OS X");
	}

	public static boolean isX86_64() {
		return System.getProperty("os.arch").contains("64");
	}
	   
}

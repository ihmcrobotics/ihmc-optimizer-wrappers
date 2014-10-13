package us.ihmc.convexOptimization;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashSet;

import com.sun.jna.Native;


public class NativeLibraryLoader {

	   public final static String LIBRARY_LOCATION = new File(System.getProperty("user.home"), ".ihmc" + File.separator + "lib").getAbsolutePath();
	   
	   
	   private NativeLibraryLoader()
	   {
	      // Disallow construction
	   }
	   
	   /* these are tailored with CMake default dynamic library names */
	   private static String get_OSX_MAC(String baseName)
	   {
		   return "lib" + baseName + "_rel.dylib";
	   }
	   private static String get_LINUX_64(String baseName)
	   {
		   return "lib" + baseName + "_rel.so";
	   }
	   private static String  get_WINDOWS_64(String baseName)
	   {
		   return baseName + "_rel.dll";
	   }
	   
	   private static final HashSet<String> loadedLibraries = new HashSet<String>();
	   
	   
	   public static String getOSDependentName(String baseName)
	   {
	      if(isX86_64())
	      {
	         if(isLinux())
	         {
	            return get_LINUX_64(baseName);
	         }
	         else if (isMac())
	         {
	            return get_OSX_MAC(baseName);
	         }
	         else if (isWindows())
	         {
	            return get_WINDOWS_64(baseName);
	         }
	      }
	      
	      throw new RuntimeException(System.getProperty("os.name") + "/" + System.getProperty("os.arch")
	            + " unsupported. Only 64bit Linux/Mac/Windows supported for now.");
	   }
	   
	   
	   public synchronized static void loadLibraryFromClassPath(String library, Class<?> interfaceClass)
	   {
	      if(loadedLibraries.contains(library))
	      {
	         return;
	      }
	      File directory = new File(LIBRARY_LOCATION);
	      if(!directory.exists())
	      {
	         directory.mkdirs();
	      }
	      File lib = new File(directory, library);
//	      if(!lib.exists())
	      {
	         InputStream stream = NativeLibraryLoader.class.getClassLoader().getResourceAsStream(library);
	         writeStreamToFile(stream, lib);
	         
	         try
	         {
	            stream.close();
	         }
	         catch (IOException e)
	         {
	         }
	      }
	      
	      try{
                      Native.register(interfaceClass, lib.getAbsolutePath());
	      }
	      catch(NoSuchMethodError e)
	      {
	    	  throw new RuntimeException("NativeLibraryLoader: JNA too old");
	      }
	      loadedLibraries.add(library);
	   }
	   
	   public static void writeStreamToFile(InputStream stream, File file)
	   {
	      try
	      {
	         FileOutputStream out = new FileOutputStream(file);
	         byte[] buf = new byte[1024];
	         int len;
	         while((len = stream.read(buf)) > 0)
	         {
	            out.write(buf, 0, len);
	         }
	         
	         out.close();
	         
	      }
	      catch (IOException e)
	      {
	         throw new RuntimeException(e);
	      }
	      
	   }

	   public static boolean isWindows()
	   {
	      return System.getProperty("os.name").startsWith("Windows");
	   }

	   public static boolean isLinux()
	   {
	      return System.getProperty("os.name").equals("Linux");
	   }

	   public static boolean isMac()
	   {
	      return System.getProperty("os.name").equals("MacOSX") || System.getProperty("os.name").equals("Mac OS X");
	   }
	   
	   public static boolean isX86_64()
	   {
	      return System.getProperty("os.arch").contains("64");
	   }
}

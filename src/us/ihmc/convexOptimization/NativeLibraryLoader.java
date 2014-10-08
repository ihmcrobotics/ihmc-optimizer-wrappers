package us.ihmc.convexOptimization;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashSet;

import com.sun.jna.Native;


public class NativeLibraryLoader {

	   public final static String LIBRARY_LOCATION = new File(System.getProperty("user.home"), ".ihmc" + File.separator + "lib").getAbsolutePath();
	   
	   private final static String OASES_MAC_64 = "libOASESConstrainedQPSolver_rel.dylib";
	   private final static String OASES_LINUX_64 = "libOASESConstrainedQPSolver_rel.so";
	   private final static String OASES_WINDOWS_64 = "OASESConstrainedQPSolver_rel.dll";
	   private static final HashSet<String> loadedLibraries = new HashSet<String>();
	   
	   
	   private NativeLibraryLoader()
	   {
	      // Disallow construction
	   }
	   
	   
	   private static String getOASESName()
	   {
	      if(isX86_64())
	      {
	         if(isLinux())
	         {
	            return OASES_LINUX_64;
	         }
	         else if (isMac())
	         {
	            return OASES_MAC_64;
	         }
	         else if (isWindows())
	         {
	            return OASES_WINDOWS_64;
	         }
	      }
	      
	      throw new RuntimeException(System.getProperty("os.name") + "/" + System.getProperty("os.arch")
	            + " unsupported. Only 64bit Linux/Mac/Windows supported for now.");
	   }
	   
	   public static void loadOASES()
	   {
		   String libOASES = getOASESName();
		   loadLibraryFromClassPath(libOASES);
	   }
	   
	   
	   private synchronized static void loadLibraryFromClassPath(String library)
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
	      if(!lib.exists())
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
	      
//	      System.setProperty("jna.library.path", lib.getAbsolutePath());
//	      System.out.println(lib.getAbsolutePath());
//	      Native.register(lib.getAbsolutePath());
//		  System.load(lib.getAbsolutePath());         
	      System.setProperty("jna.library.path", lib.getParentFile().getAbsolutePath());
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

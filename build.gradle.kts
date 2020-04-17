plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.8"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.30"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-optimizer-wrappers"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:dense64:0.30")
   api("us.ihmc:ihmc-native-library-loader:1.2.1")
   api("net.java.dev.jna:jna:5.2.0")
}

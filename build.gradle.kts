plugins {
   id("us.ihmc.ihmc-build") version "0.23.0"
   id("us.ihmc.ihmc-ci") version "7.3"
   id("us.ihmc.ihmc-cd") version "1.17"
}

ihmc {
   group = "us.ihmc"
   version = "0.0.31"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-optimizer-wrappers"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("net.java.dev.jna:jna:5.2.0")

   api("us.ihmc:ihmc-native-library-loader:1.2.1")
}

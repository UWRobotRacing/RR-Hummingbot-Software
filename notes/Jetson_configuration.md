## JETSON

### Tools:

1. `sudo ./tegrastats` to show cpu & mem states

## DEVICES

### PCI-E Inateck USB 3.0

1. No other driver required
2. ==Just need to **connect** both power and data bus before Jetson boots up==



## JETPACK

### Installation

1. Connect to a router !!! ==(It is painful without a router)==

2. Follow instructions:

   1. JetPack 3.3 and it's installation guide are listed in the Jetson Download Center

      <https://developer.nvidia.com/embedded/downloads>

      Here is a direct link to the JetPack 3.3 installation guide.

      <https://developer.nvidia.com/embedded/dlc/jetpack-install-guide-3_3>

      

## NETWORK

### Setup network share with router

1. **Jetson TX2**: 
   1. Wifi_setting -> Edit Connections -> (Create a new ethernet connection) 
   2. Ethernet (Device: eth0) 
   3. IPv4 settings (Shared to other computers)
2. **Router**: 
   1. Connect to Hummingbot Wifi 
   2. Browse to http://tplinklogin.net/
   3. Mode Setting: Router

### VNC

1. Resources:

   1. https://help.ubuntu.com/community/VNC/Servers 

2. **VINO** (Built-in)

   1. Search **Desktop Sharing**

      1. Enable screen sharing

      2. (Option* for mac):

         > There's an **encryption incompatibility** between Vino ans Mac, which manifests in both the built-in OSX VNC client and Chicken of the VNC. I had to disable encryption to get it to work: 
         >
         > `sudo apt-get -y install dconf-tools`
         >
         > `dconf write /org/gnome/desktop/remote-access/require-encryption false`

   2. Client - MAC:

      1. Spotlight: search **Screen Sharing.app**
      2. `vnc://10.42.0.1:5900`

3. #### =[Dummy Desktop](https://www.bonusbits.com/wiki/HowTo:Fix_Ubuntu_Desktop_to_Boot_without_Monitor_Connected)== ####

   1. Install dummy package

      ```bash
      sudo apt-get install xserver-xorg-video-dummy
      ```

   2. Save Default & Create new X Windows Configuration File

      ```bash
      sudo mv /etc/X11/xorg.conf /etc/X11/xorg.conf.bak
      sudo nano /etc/X11/xorg.conf
      ```

   3. [Add the following content to the file](https://devtalk.nvidia.com/default/topic/995621/jetson-tx1/jetson-tx1-desktop-sharing-resolution-problem-without-real-monitor/)

      1. Set the resolution to what you like (whatever resolution the screen is that is used to connect remotely is probably is a good idea)

         ```bash
         # Copyright (c) 2011-2013 NVIDIA CORPORATION.  All Rights Reserved.
         
         #
         # This is the minimal configuration necessary to use the Tegra driver.
         # Please refer to the xorg.conf man page for more configuration
         # options provided by the X server, including display-related options
         # provided by RandR 1.2 and higher.
         
         # Disable extensions not useful on Tegra.
         Section "Module"
             Disable     "dri"
             SubSection  "extmod"
                 Option  "omit xfree86-dga"
             EndSubSection
         EndSection
         
         
         Section "Device"
             Identifier  "Tegra0"
             Driver      "nvidia"
             Option      "AllowEmptyInitialConfiguration" "true"
         EndSection
         
         Section "Monitor"
            Identifier "DSI-0"
            Option    "Ignore"
         EndSection
         
         Section "Screen"
            Identifier    "Default Screen"
            Monitor        "Configured Monitor"
            Device        "Default Device"
            SubSection "Display"
                Depth    24
                Virtual 1920 1080
            EndSubSection
         EndSection
         ```

   4. Save the file






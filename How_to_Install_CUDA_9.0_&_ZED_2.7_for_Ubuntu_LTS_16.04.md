# How to Install CUDA 9.0 & ZED 2.7 for Ubuntu LTS 16.04
*Updated as of Feb 1st, 2019* 
Written by: Andrew Jin
****

Please Note: If you have made previous attempts and have failed in whatever reason, it may affect the installation steps defined by this documentation.

### Installation Guide

#### Please note: You do need a graphics driver that can run CUDA so check that first with googling :D
#### If you already have an nvidia graphics driver installed skip to step 5
1. Run command (This will add the nvidia graphics repository to your system)
```sh
$ sudo add-apt-repository ppa:graphics-drivers/ppa
$ sudo apt-get update
```
2. Go into systems settings -> software & updates -> Additional Drivers tab -> look for NVIDIA Binary Driver "version number"
3. After you find the version number (if you see multiple version on your system choose the highest value) run this command
```sh
$ sudo apt-get install nvidia-'version-number'
```
* Ensure the 'version-number' on this command is the one that is visible from your system settings
4. After this reboot your computer and in order to check that your driver is properly installed run:
```sh
$ nvidia-smi
```
5. Now that you have the graphics driver appropritate to install CUDA-9.0
    This should be installed first as ZED will require you to have CUDA-9.0 working
    Install link: [CUDA](https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux)
    Choose the following options:
    - Operating System: Linux
    - Architecture: x86_64
    - Distribution: Ubuntu
    - Version: 16.04
    - Installer Type: deb (local)
    After these options are chosen, download the base installer and wait for the installation to finish
6. Now that you have the debian package (this will be located in your Downloads directory if you havent changed your default download file path), we can finally install CUDA onto your linux environment. Using these appropriate commands:
```sh
$ cd Downloads //to go into the directory of Downloads
$ sudo dpkg -i cuda-repo-ubuntu1604-9-0-local_9.0.176-1_amd64.deb
```
Follow the instructions provided by cuda. To speed up this process:
    - Feel free to read through the Terms of Agreement but if youre lazy like me press 'q' and it will skip to the next instructions
    - If it asks you to accept something just type 'accept'
    - Generally if it asks for file path or shows a default value for the question just go with whatever is defaulted by pressing enter without any other input
7. Now that you have reached this stage all you have to run to finish installing cuda is one more command:
```sh
$ sudo apt-get install cuda
```
Now let cuda do its thing magically and after it is done for good measure reboot your computer just so that you dont get random hiccups. If it cant find the cuda package for whatever reason just make sure to run apt-get update before trying the command again.
8. Now it is time to install the zed package.
    - Install: [ZED 2.7](https://download.stereolabs.com/zedsdk/2.7/ubuntu16_cuda9)
9. After the installation package finishes downloading enter the following commands:
```sh
$ cd Downloads //If you killed your terminal for whatever reason before
$ chmod +x ZED_SDK_Ubuntu16_CUDA9_v2.7.1.run
$ ./ZED_SDK_Ubuntu16_CUDA9_v2.7.1.run
```
10. The following commands will run the .run file and now just follow whatever ZED asks you to do in terms of steps. The **most important thing** you're looking for is some output from that says something along the lines of **CUDA 9 Found**. This will basically mean that you have CUDA 9 on your system for sure and pray to something that ZED installation wont fail. If you didnt find the **most important thing** show up in your output and you get a red error message saying something along the lines of **CUDA 9 is required** then you need to retrace your steps and go install cuda again.
11. If Nothing else fails and you basically get a installation complete, congratulations you have literally done the thing that took me months to figure out in maybe an hour! Give yourself a pat on the back and go do something fun.
****
![fiesta-parrot](https://i.kym-cdn.com/photos/images/original/001/240/446/681.gif)


Utility to process speech for VR demo in Factorio.

## Installation guide

### Install Anaconda

*  install Anaconda [https://docs.anaconda.com/anaconda/install/windows/](https://docs.anaconda.com/anaconda/install/windows/) 
*  spustiť "install.bat". Ten vytvorí conda prostredie, kde sa nainštalujú potrebné balíčky a stiahnu dodatočné dáta.


Potom by na spustenie malo stačiť:

```
conda activate nlp 
```
### Install google speech:
```
pip install google_speech
pip install inputimeout
```


### Install sox:
* Download the release version: 14.4.2: [https://sourceforge.net/projects/sox/files/sox/](https://sourceforge.net/projects/sox/files/sox/)
* Run the setup file downloaded and complete the installation.
* Add the installation directory to SYSTEM PATH variable, if you don't know how to do this, follow the guide [here](https://docs.alfresco.com/4.2/tasks/fot-addpath.html).
* MP3 encodings cannot be decoded without a decoder. We will use: MAD decoder library ([https://sourceforge.net/projects/mad/](https://sourceforge.net/projects/mad/)), therefore the dynamic libraries must be installed. Download the following file attached and place the contents(.dll files) inside the SoX installation folder.
  * Download the files here: [libmad-0.dll_and_libmp3lame-0.dll.7z](https://at.projects.genivi.org/wiki/download/attachments/40403961/libmad-0.dll_and_libmp3lame-0.dll.7z?version=1&modificationDate=1563087469000&api=v2)
* Now you are good to go. Verify the SoX installation by typing in the following command in a command prompt and checking whether it prints out the version of SoX installed.
 ```
sox --version
```

## Run 
go to the folder
```
python setup.py clean
python setup.py build
python -m speech_vr
```
### Local testing:
run server: 
```
python server.py
```
run speech VR demo locally:
```
python -m speech_vr local
```




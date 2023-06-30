# ofxAravis

Fork of a fork of ofxAaravis - a wrapper for [Aravis library](https://github.com/AravisProject/aravis) - a vision library for **Genicam / USB3 ML cameras**.

* This fork is adapted to work on **M1 macOS** using [brew](https://brew.sh/).
* For **x86 macOS**, the addon_config.mk can be edited to point from **/opt/homebrew** to **/usr/local** - see [this fork](https://github.com/bltzr/ofxAravis). 
* Aravis and ara-viewer can also be easily built on macOS following [build instructions](https://aravisproject.github.io/aravis/building.html). 
* There is a basic example and a utility for routing devices to **Syphon** - [AravisSyphonServer](https://github.com/autr/AravisSyphonServer). 
* For the Syphon app, you must use a more recent fork of [ofxSyphon](https://github.com/autr/ofxSyphon) and disable **ARC** in Xcode by setting flag `-fno-objc-arc` on all OBJ-C files in **Build Phases > Compile Source**.
* Tested with a **FLIR Blackfly S** on macOS Ventura and Debian 10.

## Installation

```
# macOS
brew install aravis glib
```

For macOS Syphon app add `-fno-objc-arc` to all .mm files in Build Phases > Compile Source.
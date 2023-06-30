# ofxAravis

Adapted to work on **M1** macOS (as well as original Linux), using [brew](https://brew.sh/).

For **x86** macOS, addon_config.mk will need to be edited to point from */opt/homebrew* to */usr/local*). See [forked repo](https://github.com/bltzr/ofxAravis).

## Installation

```
brew install aravis glib
```

Optionally Aravis can be built on macOS easily following [these instructions](https://aravisproject.github.io/aravis/building.html).

## Examples

There is a basic example and also an app which routes all Aravis devices to Syphon using max FPS and sensor size.
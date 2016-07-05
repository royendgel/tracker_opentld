# Introduction

This is a C++ implementation of a tracker on CARMEN with OpenTLD that was originally published in MATLAB by Zdenek Kalal. This OpenTLD implementation was published by (https://github.com/gnebehay/OpenTLD) is used for tracking objects in video streams. What makes this algorithm outstanding is that it does not make use of any training data. This implementation is based solely on open source libraries, meaning that you do not need any commercial products to compile or run it.

To know more about the OpenTLD by gnebehay this documentation of the internals as well as other possibly helpful information is contained in this [master thesis](https://github.com/downloads/gnebehay/OpenTLD/gnebehay_thesis_msc.pdf).

# Usage
## Keyboard shortcuts

* `q` quit
* `b` remember current frame as background model / clear background
* `c` clear model and stop tracking  git
* `l` toggle learning
* `a` toggle alternating mode (if true, detector is switched off when tracker is available)
* `r` clear model, let user reinit tracking

## Command line options
### Synopsis
`tracker_opentld [camera_number] [camera_side (0-left; 1-right)]`

# Building
## Dependencies
* OpenCV
* libconfig++ (optional)

## Compiling
### Linux (make)
Navigate with the terminal to the build directory
* `make` build the project
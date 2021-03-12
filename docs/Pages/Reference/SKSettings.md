---
layout: default
title: SKSettings
description: StereoKit initialization settings! Setup SK.settings with your data before calling SK.Initialize.
---
# SKSettings

StereoKit initialization settings! Setup SK.settings with
your data before calling SK.Initialize.


## Instance Fields and Properties

|  |  |
|--|--|
|[DepthMode]({{site.url}}/Pages/Reference/DepthMode.html) [depthMode]({{site.url}}/Pages/Reference/SKSettings/depthMode.html)|What kind of depth buffer should StereoKit use? A fast one, a detailed one, one that uses stencils? By default, StereoKit uses a balanced mix depending on platform, prioritizing speed but opening up when there's headroom.|
|bool [disableFlatscreenMRSim]({{site.url}}/Pages/Reference/SKSettings/disableFlatscreenMRSim.html)|By default, StereoKit will simulate Mixed Reality input so developers can test MR spaces without being in a headeset. If You don't want this, you can disable it with this setting!|
|[DisplayMode]({{site.url}}/Pages/Reference/DisplayMode.html) [displayPreference]({{site.url}}/Pages/Reference/SKSettings/displayPreference.html)|Which display type should we try to load? Default is `DisplayMode.MixedReality`.|
|int [flatscreenHeight]({{site.url}}/Pages/Reference/SKSettings/flatscreenHeight.html)|If using Runtime.Flatscreen, the pixel size of the window on the screen.|
|int [flatscreenPosX]({{site.url}}/Pages/Reference/SKSettings/flatscreenPosX.html)|If using Runtime.Flatscreen, the pixel position of the window on the screen.|
|int [flatscreenPosY]({{site.url}}/Pages/Reference/SKSettings/flatscreenPosY.html)|If using Runtime.Flatscreen, the pixel position of the window on the screen.|
|int [flatscreenWidth]({{site.url}}/Pages/Reference/SKSettings/flatscreenWidth.html)|If using Runtime.Flatscreen, the pixel size of the window on the screen.|
|bool [noFlatscreenFallback]({{site.url}}/Pages/Reference/SKSettings/noFlatscreenFallback.html)|If the preferred display fails, should we avoid falling back to flatscreen and just crash out? Default is false.|



## Static Fields and Properties

|  |  |
|--|--|
|string [appName]({{site.url}}/Pages/Reference/SKSettings/appName.html)|Name of the application, this shows up an the top of the Win32 window, and is submitted to OpenXR. OpenXR caps this at 128 characters.|
|string [assetsFolder]({{site.url}}/Pages/Reference/SKSettings/assetsFolder.html)|Where to look for assets when loading files! Final path will look like '[assetsFolder]/[file]', so a trailing '/' is unnecessary.|


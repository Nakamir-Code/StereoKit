using System;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;

namespace StereoKit
{
	static class NativeLib
	{
		static bool _loaded = false;

		internal static bool Load()
		{
			if (_loaded)
				return true;

			// Browsers have their own strategy for linking
			if (RuntimeInformation.OSDescription == "Browser")
				return true;

			NativeLibrary.SetDllImportResolver(
				typeof(NativeLib).Assembly,
				ResolveStereoKit);

			_loaded = true;
			return true;
		}

		static nint ResolveStereoKit(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
		{
			if (libraryName != "StereoKitC")
				return 0;

			// The default resolver handles runtimes/{rid}/native/ automatically
			//if (NativeLibrary.TryLoad(libraryName, assembly, searchPath, out nint handle))
			//	return handle;
			nint handle = 0;

			// Fallback: try platform-specific paths from the app base directory
			string arch = RuntimeInformation.OSArchitecture == Architecture.Arm64 ? "arm64" : "x64";
			string basePath = AppContext.BaseDirectory;

			if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
				NativeLibrary.TryLoad(Path.Combine(basePath, "runtimes", $"win-{arch}", "native", "StereoKitC.dll"), out handle);
			else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
				NativeLibrary.TryLoad(Path.Combine(basePath, "runtimes", $"linux-{arch}", "native", "libStereoKitC.so"), out handle);
			else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
				NativeLibrary.TryLoad(Path.Combine(basePath, "runtimes", $"osx-{arch}", "native", "libStereoKitC.dylib"), out handle);

			return handle;
		}
	}
}

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace StereoKit
{
	/// <summary>For use with World.FromSpatialNode, this indicates the type of
	/// node that's being bridged with OpenXR.</summary>
	public enum SpatialNodeType 
	{
		/// <summary>Static spatial nodes track the pose of a fixed location in
		/// the world relative to reference spaces. The tracking of static
		/// nodes may slowly adjust the pose over time for better accuracy but
		/// the pose is relatively stable in the short term, such as between
		/// rendering frames. For example, a QR code tracking library can use a
		/// static node to represent the location of the tracked QR code.
		/// </summary>
		Static,
		/// <summary>Dynamic spatial nodes track the pose of a physical object
		/// that moves continuously relative to reference spaces. The pose of
		/// dynamic spatial nodes can be very different within the duration of
		/// a rendering frame. It is important for the application to use the
		/// correct timestamp to query the space location. For example, a color
		/// camera mounted in front of a HMD is also tracked by the HMD so a
		/// web camera library can use a dynamic node to represent the camera
		/// location.</summary>
		Dynamic
	}

	/// <summary>World contains information about the real world around the 
	/// user. This includes things like play boundaries, scene understanding,
	/// and other various things.</summary>
	public static class World
	{
		/// <summary>This refers to the play boundary, or guardian system
		/// that the system may have! Not all systems have this, so it's
		/// always a good idea to check this first!</summary>
		public static bool HasBounds  => NativeAPI.world_has_bounds();
		/// <summary>This is the size of a rectangle within the play
		/// boundary/guardian's space, in meters if one exists. Check
		/// `World.BoundsPose` for the center point and orientation of the
		/// boundary, and check `World.HasBounds` to see if it exists at all!
		/// </summary>
		public static Vec2 BoundsSize => NativeAPI.world_get_bounds_size();
		/// <summary>This is the orientation and center point of the system's
		/// boundary/guardian. This can be useful to find the floor height!
		/// Not all systems have a boundary, so be sure to check 
		/// `World.HasBounds` first.</summary>
		public static Pose BoundsPose => NativeAPI.world_get_bounds_pose();

		/// <summary>The mode or "reference space" that StereoKit uses for
		/// determining its base origin. This is determined by the initial
		/// value provided in SKSettings.origin, as well as by support from the
		/// underlying runtime. The mode reported here will _not_ necessarily
		/// be the one requested in initialization, as fallbacks are
		/// implemented using different available modes.</summary>
		public static OriginMode OriginMode => NativeAPI.world_get_origin_mode();

		/// <summary>This reports the status of the device's positional
		/// tracking. If the room is too dark, or a hand is covering tracking
		/// sensors, or some other similar 6dof tracking failure, this would
		/// report as not tracked.
		/// 
		/// Note that this does not factor in the status of rotational
		/// tracking. Rotation is typically done via gyroscopes/accelerometers,
		/// which don't really fail the same way positional tracking system
		/// can.</summary>
		public static BtnState Tracked => NativeAPI.world_get_tracked();

		/// <summary>This is relative to the base reference point and is NOT
		/// in world space! The origin StereoKit uses is actually a base
		/// reference point combined with an offset! You can use this to read
		/// or set the offset from the OriginMode reference point. </summary>
		public static Pose OriginOffset
		{
			get => NativeAPI.world_get_origin_offset();
			set => NativeAPI.world_set_origin_offset(value);
		}

		/// <summary>What information should StereoKit use to determine when
		/// the next world data refresh happens? See the `WorldRefresh` enum
		/// for details.</summary>
		public static WorldRefresh RefreshType { 
			get => NativeAPI.world_get_refresh_type(); 
			set => NativeAPI.world_set_refresh_type(value); }

		/// <summary>Radius, in meters, of the area that StereoKit should
		/// scan for world data. Default is 4. When using the 
		/// `WorldRefresh.Area` refresh type, the world data will refresh
		/// when the user has traveled half this radius from the center of
		/// where the most recent refresh occurred. </summary>
		public static float RefreshRadius {
			get => NativeAPI.world_get_refresh_radius();
			set => NativeAPI.world_set_refresh_radius(value); }

		/// <summary>The refresh interval speed, in seconds. This is only
		/// applicable when using `WorldRefresh.Timer` for the refresh type.
		/// Note that the system may not be able to refresh as fast as you
		/// wish, and in that case, StereoKit will always refresh as soon as 
		/// the previous refresh finishes.</summary>
		public static float RefreshInterval {
			get => NativeAPI.world_get_refresh_interval();
			set => NativeAPI.world_set_refresh_interval(value);
		}

		/// <summary>Converts a Windows Mirage spatial node GUID into a Pose
		/// based on its current position and rotation! Check
		/// SK.System.spatialBridgePresent to see if this is available to
		/// use. Currently only on HoloLens, good for use with the Windows
		/// QR code package.</summary>
		/// <param name="spatialNodeGuid">A Windows Mirage spatial node GUID
		/// acquired from a windows MR API call.</param>
		/// <param name="spatialNodeType">Type of spatial node to locate.</param>
		/// <param name="qpcTime">A windows performance counter timestamp at
		/// which the node should be located, obtained from another API or
		/// with System.Diagnostics.Stopwatch.GetTimestamp().</param>
		/// <returns>A Pose representing the current orientation of the
		/// spatial node.</returns>
		public static Pose FromSpatialNode(Guid spatialNodeGuid, SpatialNodeType spatialNodeType = SpatialNodeType.Static, long qpcTime = 0)
			=> NativeAPI.world_from_spatial_graph(spatialNodeGuid.ToByteArray(), spatialNodeType == SpatialNodeType.Dynamic, qpcTime);

		/// <summary>Converts a Windows Mirage spatial node GUID into a Pose
		/// based on its current position and rotation! Check
		/// SK.System.spatialBridgePresent to see if this is available to
		/// use. Currently only on HoloLens, good for use with the Windows
		/// QR code package.</summary>
		/// <param name="spatialNodeGuid">A Windows Mirage spatial node GUID
		/// acquired from a windows MR API call.</param>
		/// <param name="pose">A resulting Pose representing the current
		/// orientation of the spatial node.</param>
		/// <param name="spatialNodeType">Type of spatial node to locate.</param>
		/// <param name="qpcTime">A windows performance counter timestamp at
		/// which the node should be located, obtained from another API or
		/// with System.Diagnostics.Stopwatch.GetTimestamp().</param>
		/// <returns>True if FromSpatialNode succeeded, and false if it failed.
		/// </returns>
		public static bool FromSpatialNode(Guid spatialNodeGuid, out Pose pose, SpatialNodeType spatialNodeType = SpatialNodeType.Static, long qpcTime = 0)
			=> NativeAPI.world_try_from_spatial_graph(spatialNodeGuid.ToByteArray(), spatialNodeType == SpatialNodeType.Dynamic, qpcTime, out pose);

		/// <summary>Converts a Windows.Perception.Spatial.SpatialAnchor's pose
		/// into SteroKit's coordinate system. This can be great for
		/// interacting with some of the UWP spatial APIs such as WorldAnchors.
		/// 
		/// This method only works on UWP platforms, check 
		/// SK.System.perceptionBridgePresent to see if this is available.
		/// </summary>
		/// <param name="perceptionSpatialAnchor">A valid
		/// Windows.Perception.Spatial.SpatialAnchor.</param>
		/// <returns>A Pose representing the current orientation of the
		/// SpatialAnchor.</returns>
		[Obsolete("UWP is no longer supported")]
		public static Pose FromPerceptionAnchor(object perceptionSpatialAnchor)
		{
			if (!OperatingSystem.IsWindows()) return Pose.Identity;
			IntPtr unknown = Marshal.GetIUnknownForObject(perceptionSpatialAnchor);
			Pose   result  = NativeAPI.world_from_perception_anchor(unknown);
			Marshal.Release(unknown);
			return result;
		}

		/// <summary>Converts a Windows.Perception.Spatial.SpatialAnchor's pose
		/// into SteroKit's coordinate system. This can be great for
		/// interacting with some of the UWP spatial APIs such as WorldAnchors.
		/// 
		/// This method only works on UWP platforms, check 
		/// SK.System.perceptionBridgePresent to see if this is available.
		/// </summary>
		/// <param name="perceptionSpatialAnchor">A valid
		/// Windows.Perception.Spatial.SpatialAnchor.</param>
		/// <param name="pose">A resulting Pose representing the current
		/// orientation of the spatial node.</param>
		/// <returns>A Pose representing the current orientation of the
		/// SpatialAnchor.</returns>
		[Obsolete("UWP is no longer supported")]
		public static bool FromPerceptionAnchor(object perceptionSpatialAnchor, out Pose pose)
		{
			if (!OperatingSystem.IsWindows()) { pose = Pose.Identity; return false; }
			IntPtr unknown = Marshal.GetIUnknownForObject(perceptionSpatialAnchor);
			bool   result  = NativeAPI.world_try_from_perception_anchor(unknown, out pose);
			Marshal.Release(unknown);
			return result;
		}

		/// <summary>World.RaycastEnabled must be set to true first! 
		/// SK.System.worldRaycastPresent must also be true. This does a ray
		/// intersection with whatever represents the environment at the
		/// moment! In this case, it's a watertight collection of low
		/// resolution meshes calculated by the Scene Understanding
		/// extension, which is only provided by the Microsoft HoloLens
		/// runtime.</summary>
		/// <param name="ray">A world space ray that you'd like to try
		/// intersecting with the world mesh.</param>
		/// <param name="intersection">The location of the intersection, and
		/// direction of the world's surface at that point. This is only
		/// valid if the method returns true.</param>
		/// <returns>True if an intersection is detected, false if raycasting
		/// is disabled, or there was no intersection.</returns>
		public static bool Raycast(Ray ray, out Ray intersection)
			=> NativeAPI.world_raycast(ray, out intersection);

		/// <summary>Off by default. This tells StereoKit to load up and
		/// display an occlusion surface that allows the real world to
		/// occlude the application's digital content! Most systems may allow
		/// you to customize the visual appearance of this occlusion surface
		/// via the World.OcclusionMaterial.
		/// Check SK.System.worldOcclusionPresent to see if occlusion can be
		/// enabled. This will reset itself to false if occlusion isn't
		/// possible. Loading occlusion data is asynchronous, so occlusion
		/// may not occur immediately after setting this flag.</summary>
		public static bool OcclusionEnabled { 
			get => NativeAPI.world_get_occlusion_enabled();
			set => NativeAPI.world_set_occlusion_enabled(value); }

		/// <summary>Off by default. This tells StereoKit to load up 
		/// collision meshes for the environment, for use with World.Raycast.
		/// Check SK.System.worldRaycastPresent to see if raycasting can be
		/// enabled. This will reset itself to false if raycasting isn't
		/// possible. Loading raycasting data is asynchronous, so collision
		/// surfaces may not be available immediately after setting this
		/// flag.</summary>
		public static bool RaycastEnabled { 
			get => NativeAPI.world_get_raycast_enabled();
			set => NativeAPI.world_set_raycast_enabled(value); }

		/// <summary>By default, this is a black(0,0,0,0) opaque unlit
		/// material that will occlude geometry, but won't show up as visible
		/// anywhere. You can override this with whatever material you would
		/// like.</summary>
		public static Material OcclusionMaterial {
			get => new Material(NativeAPI.world_get_occlusion_material());
			set => NativeAPI.world_set_occlusion_material(value._inst); }

		/// <summary>Check if world tracking capabilities (markers, surfaces,
		/// anchors) are available on this system.</summary>
		public static bool TrackingAvailable => NativeAPI.world_tracking_is_available();

		/// <summary>Get the world tracking capabilities supported by the
		/// current system.</summary>
		public static WorldTrackingCaps TrackingCapabilities => NativeAPI.world_tracking_get_capabilities();

		/// <summary>Check if a specific marker type is supported for
		/// tracking.</summary>
		/// <param name="type">The marker type to check.</param>
		/// <returns>True if the marker type can be tracked.</returns>
		public static bool HasMarkerType(WorldMarkerType type)
			=> NativeAPI.world_tracking_has_marker_type(type);

		/// <summary>Is persistence available for tracked entities? Currently
		/// only anchors support cross-session persistence, but the capability
		/// check is designed to be type-agnostic for future expansion.</summary>
		public static bool PersistenceAvailable => NativeAPI.world_persistence_is_available();

		/// <summary>Check if a tracked type supports persistence.</summary>
		/// <typeparam name="T">Tracked type to query.</typeparam>
		/// <returns>True if this type can be persisted.</returns>
		public static bool SupportsPersistence<T>() where T : struct, ITracked
		{
			if (typeof(T) == typeof(AnchorInfo)) return PersistenceAvailable;
			return false;
		}

		/// <summary>Fires when a marker is detected or updated. Subscribe
		/// before or after calling
		/// <see cref="StartTracking(TrackingType, WorldMarkerTypes)"/>.
		/// </summary>
		public static event Action<MarkerInfo>  OnMarker;

		/// <summary>Fires when a surface is detected or updated. Subscribe
		/// before or after calling
		/// <see cref="StartTracking(TrackingType)"/>.</summary>
		public static event Action<SurfaceInfo> OnSurface;

		/// <summary>Fires when an anchor is detected or updated. Subscribe
		/// before or after calling
		/// <see cref="StartTracking(TrackingType)"/> with
		/// <see cref="TrackingType.Anchors"/>.</summary>
		public static event Action<AnchorInfo>  OnAnchor;

		/// <summary>Start tracking the specified category. For markers this
		/// enables all marker types; use the overloads to narrow the set.
		/// </summary>
		/// <param name="type">Which tracking category to enable.</param>
		/// <returns>True if tracking started successfully.</returns>
		public static bool StartTracking(TrackingType type)
			=> StartTracking(type, WorldMarkerTypes.All);

		/// <summary>Start tracking markers with a specific set of marker
		/// types. Ignored when <paramref name="type"/> is not
		/// <see cref="TrackingType.Markers"/>.</summary>
		/// <param name="type">Which tracking category to enable.</param>
		/// <param name="markerTypes">Bitmask of marker types to track.
		/// </param>
		/// <returns>True if tracking started successfully.</returns>
		public static bool StartTracking(TrackingType type, WorldMarkerTypes markerTypes)
		{
			switch (type) {
			case TrackingType.Markers:  return NativeAPI.world_marker_start (markerTypes, _nativeMarkerCb, IntPtr.Zero);
			case TrackingType.Surfaces: return NativeAPI.world_surface_start(_nativeSurfaceCb, IntPtr.Zero);
			case TrackingType.Anchors:  return NativeAPI.world_anchor_start (_nativeAnchorCb, IntPtr.Zero);
			default: return false;
			}
		}

		/// <summary>Start tracking markers with explicit dictionary
		/// configuration.</summary>
		/// <param name="config">Marker configuration including types and
		/// dictionary selection.</param>
		/// <returns>True if tracking started successfully.</returns>
		public static bool StartTracking(MarkerConfig config)
			=> NativeAPI.world_marker_start_ex(config, _nativeMarkerCb, IntPtr.Zero);

		/// <summary>Stop tracking a category.</summary>
		/// <param name="type">Which tracking category to disable.</param>
		public static void StopTracking(TrackingType type)
		{
			switch (type) {
			case TrackingType.Markers:  NativeAPI.world_marker_stop();  break;
			case TrackingType.Surfaces: NativeAPI.world_surface_stop(); break;
			case TrackingType.Anchors:  NativeAPI.world_anchor_stop();  break;
			}
		}

		/// <summary>Is the given tracking category currently active?</summary>
		/// <param name="type">Category to query.</param>
		/// <returns>True if tracking is active.</returns>
		public static bool IsTracking(TrackingType type) => type switch {
			TrackingType.Markers  => NativeAPI.world_marker_is_active(),
			TrackingType.Surfaces => NativeAPI.world_surface_is_active(),
			TrackingType.Anchors  => NativeAPI.world_anchor_is_active(),
			_ => false,
		};

		/// <summary>Number of currently detected items of this tracked type.
		/// </summary>
		/// <typeparam name="T">A tracked data type such as
		/// <see cref="MarkerInfo"/>, <see cref="SurfaceInfo"/>, or
		/// <see cref="AnchorInfo"/>.</typeparam>
		/// <returns>Count of detected items.</returns>
		public static int GetTrackedCount<T>() where T : struct, ITracked
		{
			if (typeof(T) == typeof(MarkerInfo))  return NativeAPI.world_marker_get_count();
			if (typeof(T) == typeof(SurfaceInfo)) return NativeAPI.world_surface_get_count();
			if (typeof(T) == typeof(AnchorInfo))  return NativeAPI.world_anchor_get_count();
			return 0;
		}

		/// <summary>Get a tracked item by index.</summary>
		/// <typeparam name="T">Tracked data type.</typeparam>
		/// <param name="index">Zero-based index into the detected list.
		/// </param>
		/// <param name="result">The item if found.</param>
		/// <returns>True if the index was valid.</returns>
		public static bool GetTracked<T>(int index, out T result) where T : struct, ITracked
		{
			if (typeof(T) == typeof(MarkerInfo)) {
				bool ok = NativeAPI.world_marker_get_at(index, out MarkerInfo m);
				result = ok ? (T)(object)m : default;
				return ok;
			}
			if (typeof(T) == typeof(SurfaceInfo)) {
				bool ok = NativeAPI.world_surface_get_at(index, out SurfaceInfo s);
				result = ok ? (T)(object)s : default;
				return ok;
			}
			if (typeof(T) == typeof(AnchorInfo)) {
				bool ok = NativeAPI.world_anchor_get_at(index, out AnchorInfo a);
				result = ok ? (T)(object)a : default;
				return ok;
			}
			result = default;
			return false;
		}

		/// <summary>Find a tracked item by its stable numeric ID.</summary>
		/// <typeparam name="T">Tracked data type.</typeparam>
		/// <param name="id">Stable ID assigned by the tracking system.
		/// </param>
		/// <param name="result">The item if found.</param>
		/// <returns>True if found.</returns>
		public static bool FindTracked<T>(ulong id, out T result) where T : struct, ITracked
		{
			if (typeof(T) == typeof(MarkerInfo)) {
				bool ok = NativeAPI.world_marker_find_by_id(id, out MarkerInfo m);
				result = ok ? (T)(object)m : default;
				return ok;
			}
			if (typeof(T) == typeof(SurfaceInfo)) {
				bool ok = NativeAPI.world_surface_find_by_id(id, out SurfaceInfo s);
				result = ok ? (T)(object)s : default;
				return ok;
			}
			if (typeof(T) == typeof(AnchorInfo)) {
				bool ok = NativeAPI.world_anchor_find_by_id(id, out AnchorInfo a);
				result = ok ? (T)(object)a : default;
				return ok;
			}
			result = default;
			return false;
		}

		/// <summary>Find a tracked marker by its decoded data content (QR
		/// text, ArUco ID, etc). Only valid for
		/// <see cref="MarkerInfo"/>.</summary>
		/// <param name="data">The data content to search for.</param>
		/// <param name="result">The marker if found.</param>
		/// <returns>True if found.</returns>
		public static bool FindTracked(string data, out MarkerInfo result)
			=> NativeAPI.world_marker_find(data, out result);

		/// <summary>Find a tracked anchor by its persistence UUID. Only valid
		/// for <see cref="AnchorInfo"/>.</summary>
		/// <param name="uuid">The persistence UUID to search for.</param>
		/// <param name="result">The anchor if found.</param>
		/// <returns>True if found.</returns>
		public static bool FindTracked(string uuid, out AnchorInfo result)
			=> NativeAPI.world_anchor_find_by_uuid(uuid, out result);

		/// <summary>Force a refresh of tracked data for the given type
		/// without waiting for the next automatic discovery cycle.</summary>
		/// <typeparam name="T">Tracked data type.</typeparam>
		public static void RefreshTracked<T>() where T : struct, ITracked
		{
			if (typeof(T) == typeof(MarkerInfo))  NativeAPI.world_marker_refresh();
			if (typeof(T) == typeof(SurfaceInfo)) NativeAPI.world_surface_refresh();
			if (typeof(T) == typeof(AnchorInfo))  NativeAPI.world_anchor_refresh();
		}

		/// <summary>Create a new spatial anchor at the given pose through
		/// the world tracking system. The callback fires once the anchor is
		/// created with its assigned ID.</summary>
		/// <param name="pose">World-space pose for the new anchor.</param>
		/// <param name="onCreated">Called when the anchor is created, or null
		/// on failure.</param>
		public static bool CreateAnchor(Pose pose, Action<AnchorInfo?> onCreated = null)
		{
			OnAnchorCreated cb = null;
			if (onCreated != null)
				cb = (ctx, in a) => onCreated(a.id != 0 ? a : (AnchorInfo?)null);
			_anchorCreateCbs.Add(cb);
			return NativeAPI.world_anchor_create(pose, cb, IntPtr.Zero);
		}

		/// <summary>Remove a tracked anchor by its ID.</summary>
		/// <param name="anchorId">The anchor's stable ID.</param>
		/// <returns>True if the anchor was found and removed.</returns>
		public static bool RemoveAnchor(ulong anchorId)
			=> NativeAPI.world_anchor_remove(anchorId);

		/// <summary>Persist an anchor for cross-session retrieval.</summary>
		/// <param name="anchorId">The anchor's stable ID.</param>
		/// <param name="onCompleted">Called with success status and UUID
		/// on completion.</param>
		/// <returns>True if the persist request was submitted.</returns>
		public static bool PersistAnchor(ulong anchorId, Action<bool, string> onCompleted = null)
		{
			OnPersistCompleted cb = null;
			if (onCompleted != null)
				cb = (ctx, success, uuidPtr) => onCompleted(success, Marshal.PtrToStringAnsi(uuidPtr));
			_persistCbs.Add(cb);
			return NativeAPI.world_anchor_persist(anchorId, cb, IntPtr.Zero);
		}

		/// <summary>Remove persistence from an anchor.</summary>
		/// <param name="anchorId">The anchor's stable ID.</param>
		/// <param name="onCompleted">Called with success status on
		/// completion.</param>
		/// <returns>True if the unpersist request was submitted.</returns>
		public static bool UnpersistAnchor(ulong anchorId, Action<bool> onCompleted = null)
		{
			OnUnpersistCompleted cb = null;
			if (onCompleted != null)
				cb = (ctx, success) => onCompleted(success);
			_unpersistCbs.Add(cb);
			return NativeAPI.world_anchor_unpersist(anchorId, cb, IntPtr.Zero);
		}

		static readonly OnMarkerEvent  _nativeMarkerCb  = (ctx, in m) => OnMarker?.Invoke(m);
		static readonly OnSurfaceEvent _nativeSurfaceCb = (ctx, in s) => OnSurface?.Invoke(s);
		static readonly OnAnchorEvent  _nativeAnchorCb  = (ctx, in a) => OnAnchor?.Invoke(a);

		// prevent GC of callback delegates passed to native code
		static readonly List<OnAnchorCreated>      _anchorCreateCbs = new List<OnAnchorCreated>();
		static readonly List<OnPersistCompleted>   _persistCbs      = new List<OnPersistCompleted>();
		static readonly List<OnUnpersistCompleted> _unpersistCbs    = new List<OnUnpersistCompleted>();
	}
}

/* SPDX-License-Identifier: MIT */
/* The authors below grant copyright rights under the MIT license:
 * Copyright (c) 2026 Nick Klingensmith
 * Copyright (c) 2026 Qualcomm Technologies, Inc.
 */

// This implements XR_EXT_spatial_entity
// https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#XR_EXT_spatial_entity

#include "ext_management.h"
#include "future.h"
#include "../openxr.h"
#include "../../asset_types/anchor.h"
#include "../../libraries/array.h"
#include "../../sk_memory.h"

#include <string.h>

#define XR_EXT_FUNCTIONS( X )                        \
	X(xrEnumerateSpatialCapabilitiesEXT)             \
	X(xrEnumerateSpatialCapabilityComponentTypesEXT) \
	X(xrEnumerateSpatialCapabilityFeaturesEXT)       \
	X(xrCreateSpatialContextAsyncEXT)                \
	X(xrCreateSpatialContextCompleteEXT)             \
	X(xrDestroySpatialContextEXT)                    \
	X(xrCreateSpatialDiscoverySnapshotAsyncEXT)      \
	X(xrCreateSpatialDiscoverySnapshotCompleteEXT)   \
	X(xrQuerySpatialComponentDataEXT)                \
	X(xrDestroySpatialSnapshotEXT)                   \
	X(xrCreateSpatialEntityFromIdEXT)                \
	X(xrDestroySpatialEntityEXT)                     \
	X(xrCreateSpatialUpdateSnapshotEXT)              \
	X(xrGetSpatialBufferStringEXT)                   \
	X(xrGetSpatialBufferUint8EXT)                    \
	X(xrGetSpatialBufferUint16EXT)                   \
	X(xrGetSpatialBufferUint32EXT)                   \
	X(xrGetSpatialBufferVector2fEXT)                 \
	X(xrGetSpatialBufferVector3fEXT)                 \
	X(xrCreateSpatialAnchorEXT)
OPENXR_DEFINE_FN_STATIC(XR_EXT_FUNCTIONS);
#define XR_PERSISTENCE_FUNCTIONS( X )                \
	X(xrCreateSpatialPersistenceContextAsyncEXT)     \
	X(xrCreateSpatialPersistenceContextCompleteEXT)  \
	X(xrDestroySpatialPersistenceContextEXT)         \
	X(xrPersistSpatialEntityAsyncEXT)                \
	X(xrPersistSpatialEntityCompleteEXT)             \
	X(xrUnpersistSpatialEntityAsyncEXT)              \
	X(xrUnpersistSpatialEntityCompleteEXT)
OPENXR_DEFINE_FN_STATIC(XR_PERSISTENCE_FUNCTIONS);

namespace sk {
typedef enum spatial_category_ {
	spatial_category_marker = 0,
	spatial_category_plane  = 1,
	spatial_category_anchor = 2,
	spatial_category_max
} spatial_category_;

typedef struct xr_spatial_context_t {
	XrSpatialContextEXT context;
	bool                context_ready;
	bool                discovery_pending;
	bool                enabled;
	uint64_t            context_id;
} xr_spatial_context_t;

typedef struct xr_tracked_marker_t {
	uint64_t               entity_id;
	uint64_t               parent_id;
	XrSpatialEntityEXT     entity_handle;
	world_marker_type_     type;
	pose_t                 pose;
	vec2                   size;
	world_tracking_state_  tracking_state;
	char*                  data;
} xr_tracked_marker_t;
typedef struct xr_tracked_plane_t {
	uint64_t               entity_id;
	uint64_t               parent_id;
	XrSpatialEntityEXT     entity_handle;
	pose_t                 pose;
	vec2                   bounds;
	vec3                   bounds3d;
	world_surface_type_    alignment;
	world_surface_label_   label;
	world_tracking_state_  tracking_state;
	array_t<vec2>          polygon;
	array_t<vec2>          mesh_vertices;
	array_t<uint32_t>      mesh_indices;
} xr_tracked_plane_t;
typedef struct xr_tracked_anchor_t {
	uint64_t               entity_id;
	uint64_t               parent_id;
	XrSpatialEntityEXT     entity_handle;
	pose_t                 pose;
	vec3                   bounds3d;
	world_tracking_state_  tracking_state;
	bool                   is_persistent;
	char                   uuid[37];
	array_t<vec3>          mesh_vertices;
	array_t<uint32_t>      mesh_indices;
} xr_tracked_anchor_t;

typedef struct xr_marker_state_t {
	world_marker_types_  enabled_types;
	aruco_dict_          aruco_dict;
	apriltag_dict_       apriltag_dict;
	void               (*callback)(void* context, const world_marker_t* marker);
	void*                callback_context;
} xr_marker_state_t;
typedef struct xr_plane_state_t {
	void (*callback)(void* context, const world_surface_t* plane);
	void*  callback_context;
} xr_plane_state_t;
typedef struct xr_anchor_state_t {
	void (*callback)(void* context, const world_anchor_t* anchor);
	void*  callback_context;
} xr_anchor_state_t;

typedef struct xr_anchor_create_request_t {
	uint64_t request_id;
	void  (*callback)(void* context, const world_anchor_t* anchor);
	void*   callback_context;
} xr_anchor_create_request_t;
typedef struct xr_persist_request_t {
	uint64_t anchor_id;
	void   (*callback_persist)(void* context, bool32_t success, const char* uuid);
	void   (*callback_unpersist)(void* context, bool32_t success);
	void*    callback_context;
} xr_persist_request_t;

struct xr_spatial_entity_state_t {
	bool                                available;
	bool                                persistence_available;
	world_tracking_caps_                capabilities;
	uint64_t                            next_context_id;
	uint64_t                            next_request_id;
	xr_spatial_context_t                contexts[spatial_category_max];
	XrSpatialPersistenceContextEXT      persistence_context;
	bool                                persistence_context_ready;
	xr_marker_state_t                   marker_state;
	xr_plane_state_t                    plane_state;
	xr_anchor_state_t                   anchor_state;
	array_t<xr_tracked_marker_t>        markers;
	array_t<xr_tracked_plane_t>         planes;
	array_t<xr_tracked_anchor_t>        anchors;
	array_t<xr_anchor_create_request_t> anchor_create_requests;
	array_t<xr_persist_request_t>       persist_requests;
};

static xr_spatial_entity_state_t local = {};
static void xr_spatial_context_begin_discovery(spatial_category_ category);
static void xr_spatial_marker_process_snapshot(XrSpatialSnapshotEXT snapshot);
static void xr_spatial_plane_process_snapshot (XrSpatialSnapshotEXT snapshot);
static void xr_spatial_anchor_process_snapshot(XrSpatialSnapshotEXT snapshot);
static void xr_spatial_create_update_snapshot (spatial_category_ category);
static void xr_fill_anchor(const xr_tracked_anchor_t& a, world_anchor_t* out);

///////////////////////////////////////////

static bool xr_marker_type_to_capability(world_marker_type_ type, XrSpatialCapabilityEXT* out_cap) {
	if (!out_cap) return false;
	switch (type) {
	case world_marker_type_qr_code:       *out_cap = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_QR_CODE_EXT;       return true;
	case world_marker_type_micro_qr_code: *out_cap = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_MICRO_QR_CODE_EXT; return true;
	case world_marker_type_aruco:         *out_cap = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_ARUCO_MARKER_EXT;  return true;
	case world_marker_type_april_tag:     *out_cap = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_APRIL_TAG_EXT;     return true;
	default: return false;
	}
}

static world_marker_type_ xr_capability_to_marker_type(XrSpatialCapabilityEXT capability, bool* out_valid) {
	if (out_valid) *out_valid = true;
	switch (capability) {
	case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_QR_CODE_EXT:       return world_marker_type_qr_code;
	case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_MICRO_QR_CODE_EXT: return world_marker_type_micro_qr_code;
	case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_ARUCO_MARKER_EXT:  return world_marker_type_aruco;
	case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_APRIL_TAG_EXT:     return world_marker_type_april_tag;
	default:
		if (out_valid) *out_valid = false;
		return world_marker_type_qr_code;
	}
}

static world_marker_types_ xr_marker_type_to_types(world_marker_type_ type) {
	switch (type) {
	case world_marker_type_qr_code:       return world_marker_types_qr;
	case world_marker_type_micro_qr_code: return world_marker_types_micro_qr;
	case world_marker_type_aruco:         return world_marker_types_aruco;
	case world_marker_type_april_tag:     return world_marker_types_april_tag;
	default: return world_marker_types_none;
	}
}

static bool xr_marker_type_in_mask(world_marker_type_ type, world_marker_types_ mask) {
	return (xr_marker_type_to_types(type) & mask) != 0;
}

static world_tracking_caps_ xr_marker_types_to_caps(world_marker_types_ types) {
	world_tracking_caps_ caps = world_tracking_caps_none;
	if (types & world_marker_types_qr)       caps = (world_tracking_caps_)(caps | world_tracking_caps_marker_qr);
	if (types & world_marker_types_micro_qr) caps = (world_tracking_caps_)(caps | world_tracking_caps_marker_micro_qr);
	if (types & world_marker_types_aruco)    caps = (world_tracking_caps_)(caps | world_tracking_caps_marker_aruco);
	if (types & world_marker_types_april_tag)caps = (world_tracking_caps_)(caps | world_tracking_caps_marker_april);
	return caps;
}

static world_tracking_state_ xr_to_tracking_state(XrSpatialEntityTrackingStateEXT state) {
	switch (state) {
	case XR_SPATIAL_ENTITY_TRACKING_STATE_TRACKING_EXT: return world_tracking_state_tracking;
	case XR_SPATIAL_ENTITY_TRACKING_STATE_PAUSED_EXT:   return world_tracking_state_paused;
	default:                                            return world_tracking_state_stopped;
	}
}

static world_surface_type_ xr_alignment_to_plane_type(XrSpatialPlaneAlignmentEXT alignment) {
	switch (alignment) {
	case XR_SPATIAL_PLANE_ALIGNMENT_HORIZONTAL_DOWNWARD_EXT: return world_surface_type_horizontal_down;
	case XR_SPATIAL_PLANE_ALIGNMENT_HORIZONTAL_UPWARD_EXT:   return world_surface_type_horizontal_up;
	case XR_SPATIAL_PLANE_ALIGNMENT_VERTICAL_EXT:            return world_surface_type_vertical;
	default:                                                 return world_surface_type_unknown;
	}
}

static world_surface_label_ xr_semantic_to_plane_label(XrSpatialPlaneSemanticLabelEXT label) {
	switch (label) {
	case XR_SPATIAL_PLANE_SEMANTIC_LABEL_FLOOR_EXT:   return world_surface_label_floor;
	case XR_SPATIAL_PLANE_SEMANTIC_LABEL_WALL_EXT:    return world_surface_label_wall;
	case XR_SPATIAL_PLANE_SEMANTIC_LABEL_CEILING_EXT: return world_surface_label_ceiling;
	case XR_SPATIAL_PLANE_SEMANTIC_LABEL_TABLE_EXT:   return world_surface_label_table;
	default:                                          return world_surface_label_unknown;
	}
}

static XrSpatialMarkerArucoDictEXT xr_aruco_dict_to_xr(aruco_dict_ dict) {
	switch (dict) {
	case aruco_dict_aruco4x4_50:   return XR_SPATIAL_MARKER_ARUCO_DICT_4X4_50_EXT;
	case aruco_dict_aruco4x4_100:  return XR_SPATIAL_MARKER_ARUCO_DICT_4X4_100_EXT;
	case aruco_dict_aruco4x4_250:  return XR_SPATIAL_MARKER_ARUCO_DICT_4X4_250_EXT;
	case aruco_dict_aruco4x4_1000: return XR_SPATIAL_MARKER_ARUCO_DICT_4X4_1000_EXT;
	case aruco_dict_aruco5x5_50:   return XR_SPATIAL_MARKER_ARUCO_DICT_5X5_50_EXT;
	case aruco_dict_aruco5x5_100:  return XR_SPATIAL_MARKER_ARUCO_DICT_5X5_100_EXT;
	case aruco_dict_aruco5x5_250:  return XR_SPATIAL_MARKER_ARUCO_DICT_5X5_250_EXT;
	case aruco_dict_aruco5x5_1000: return XR_SPATIAL_MARKER_ARUCO_DICT_5X5_1000_EXT;
	case aruco_dict_aruco6x6_50:   return XR_SPATIAL_MARKER_ARUCO_DICT_6X6_50_EXT;
	case aruco_dict_aruco6x6_100:  return XR_SPATIAL_MARKER_ARUCO_DICT_6X6_100_EXT;
	case aruco_dict_aruco6x6_250:  return XR_SPATIAL_MARKER_ARUCO_DICT_6X6_250_EXT;
	case aruco_dict_aruco6x6_1000: return XR_SPATIAL_MARKER_ARUCO_DICT_6X6_1000_EXT;
	case aruco_dict_aruco7x7_50:   return XR_SPATIAL_MARKER_ARUCO_DICT_7X7_50_EXT;
	case aruco_dict_aruco7x7_100:  return XR_SPATIAL_MARKER_ARUCO_DICT_7X7_100_EXT;
	case aruco_dict_aruco7x7_250:  return XR_SPATIAL_MARKER_ARUCO_DICT_7X7_250_EXT;
	case aruco_dict_aruco7x7_1000: return XR_SPATIAL_MARKER_ARUCO_DICT_7X7_1000_EXT;
	default:                       return XR_SPATIAL_MARKER_ARUCO_DICT_4X4_50_EXT;
	}
}

static XrSpatialMarkerAprilTagDictEXT xr_apriltag_dict_to_xr(apriltag_dict_ dict) {
	switch (dict) {
	case apriltag_dict_tag16h5:  return XR_SPATIAL_MARKER_APRIL_TAG_DICT_16H5_EXT;
	case apriltag_dict_tag25h9:  return XR_SPATIAL_MARKER_APRIL_TAG_DICT_25H9_EXT;
	case apriltag_dict_tag36h10: return XR_SPATIAL_MARKER_APRIL_TAG_DICT_36H10_EXT;
	case apriltag_dict_tag36h11: return XR_SPATIAL_MARKER_APRIL_TAG_DICT_36H11_EXT;
	default:                     return XR_SPATIAL_MARKER_APRIL_TAG_DICT_36H11_EXT;
	}
}

static bool xr_get_vec2_buffer(XrSpatialSnapshotEXT snapshot, XrSpatialBufferEXT buffer, array_t<vec2>* out_array) {
	if (!out_array) return false;
	out_array->clear();
	
	XrSpatialBufferGetInfoEXT info = { XR_TYPE_SPATIAL_BUFFER_GET_INFO_EXT };
	info.bufferId = buffer.bufferId;
	
	uint32_t count = 0;
	XrResult result = xrGetSpatialBufferVector2fEXT(snapshot, &info, 0, &count, nullptr);
	if (XR_FAILED(result) || count == 0) return false;
	
	XrVector2f* xr_vecs = sk_malloc_t(XrVector2f, count);
	result = xrGetSpatialBufferVector2fEXT(snapshot, &info, count, &count, xr_vecs);
	if (XR_FAILED(result)) {
		sk_free(xr_vecs);
		return false;
	}
	
	for (uint32_t i = 0; i < count; i++) {
		out_array->add({ xr_vecs[i].x, xr_vecs[i].y });
	}
	sk_free(xr_vecs);
	return true;
}

static bool xr_get_vec3_buffer(XrSpatialSnapshotEXT snapshot, XrSpatialBufferEXT buffer, array_t<vec3>* out_array) {
	if (!out_array) return false;
	out_array->clear();
	
	XrSpatialBufferGetInfoEXT info = { XR_TYPE_SPATIAL_BUFFER_GET_INFO_EXT };
	info.bufferId = buffer.bufferId;
	
	uint32_t count = 0;
	XrResult result = xrGetSpatialBufferVector3fEXT(snapshot, &info, 0, &count, nullptr);
	if (XR_FAILED(result) || count == 0) return false;
	
	XrVector3f* xr_vecs = sk_malloc_t(XrVector3f, count);
	result = xrGetSpatialBufferVector3fEXT(snapshot, &info, count, &count, xr_vecs);
	if (XR_FAILED(result)) {
		sk_free(xr_vecs);
		return false;
	}
	
	for (uint32_t i = 0; i < count; i++) {
		out_array->add({ xr_vecs[i].x, xr_vecs[i].y, xr_vecs[i].z });
	}
	sk_free(xr_vecs);
	return true;
}

static bool xr_get_index_buffer(XrSpatialSnapshotEXT snapshot, XrSpatialBufferEXT buffer, array_t<uint32_t>* out_array) {
	if (!out_array) return false;
	out_array->clear();
	
	XrSpatialBufferGetInfoEXT info = { XR_TYPE_SPATIAL_BUFFER_GET_INFO_EXT };
	info.bufferId = buffer.bufferId;
	
	uint32_t count = 0;
	XrResult result;
	
	// Try uint32 first
	result = xrGetSpatialBufferUint32EXT(snapshot, &info, 0, &count, nullptr);
	if (XR_SUCCEEDED(result) && count > 0) {
		uint32_t* data = sk_malloc_t(uint32_t, count);
		result = xrGetSpatialBufferUint32EXT(snapshot, &info, count, &count, data);
		if (XR_SUCCEEDED(result)) {
			for (uint32_t i = 0; i < count; i++) out_array->add(data[i]);
		}
		sk_free(data);
		return XR_SUCCEEDED(result);
	}
	
	// Try uint16
	result = xrGetSpatialBufferUint16EXT(snapshot, &info, 0, &count, nullptr);
	if (XR_SUCCEEDED(result) && count > 0) {
		uint16_t* data = sk_malloc_t(uint16_t, count);
		result = xrGetSpatialBufferUint16EXT(snapshot, &info, count, &count, data);
		if (XR_SUCCEEDED(result)) {
			for (uint32_t i = 0; i < count; i++) out_array->add((uint32_t)data[i]);
		}
		sk_free(data);
		return XR_SUCCEEDED(result);
	}
	
	// Try uint8
	result = xrGetSpatialBufferUint8EXT(snapshot, &info, 0, &count, nullptr);
	if (XR_SUCCEEDED(result) && count > 0) {
		uint8_t* data = sk_malloc_t(uint8_t, count);
		result = xrGetSpatialBufferUint8EXT(snapshot, &info, count, &count, data);
		if (XR_SUCCEEDED(result)) {
			for (uint32_t i = 0; i < count; i++) out_array->add((uint32_t)data[i]);
		}
		sk_free(data);
		return XR_SUCCEEDED(result);
	}
	
	return false;
}

///////////////////////////////////////////

static XrSpatialEntityEXT xr_entity_handle_from_id(spatial_category_ category, XrSpatialEntityIdEXT entity_id) {
	xr_spatial_context_t* ctx = &local.contexts[category];
	if (!ctx->context_ready || ctx->context == XR_NULL_HANDLE)
		return XR_NULL_HANDLE;

	XrSpatialEntityFromIdCreateInfoEXT create_info = { XR_TYPE_SPATIAL_ENTITY_FROM_ID_CREATE_INFO_EXT };
	create_info.entityId = entity_id;

	XrSpatialEntityEXT entity = XR_NULL_HANDLE;
	XrResult result = xrCreateSpatialEntityFromIdEXT(ctx->context, &create_info, &entity);
	if (XR_FAILED(result)) {
		log_diagf("xrCreateSpatialEntityFromIdEXT [%s]", openxr_string(result));
		return XR_NULL_HANDLE;
	}
	return entity;
}

static void xr_spatial_context_begin_discovery(spatial_category_ category) {
	xr_spatial_context_t* ctx = &local.contexts[category];
	if (!ctx->enabled || !ctx->context_ready || ctx->context == XR_NULL_HANDLE)
		return;

	if (ctx->discovery_pending)
		return;

	XrSpatialComponentTypeEXT components[8];
	uint32_t component_count = 0;

	switch (category) {
	case spatial_category_marker:
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_MARKER_EXT;
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_BOUNDED_2D_EXT;
		break;
	case spatial_category_plane:
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_PLANE_ALIGNMENT_EXT;
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_BOUNDED_2D_EXT;
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_PLANE_SEMANTIC_LABEL_EXT;
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_POLYGON_2D_EXT;
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_MESH_2D_EXT;
		break;
	case spatial_category_anchor:
		components[component_count++] = XR_SPATIAL_COMPONENT_TYPE_ANCHOR_EXT;
		break;
	default:
		return;
	}

	XrSpatialDiscoverySnapshotCreateInfoEXT info = { XR_TYPE_SPATIAL_DISCOVERY_SNAPSHOT_CREATE_INFO_EXT };
	info.componentTypeCount = component_count;
	info.componentTypes     = components;

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrCreateSpatialDiscoverySnapshotAsyncEXT(ctx->context, &info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialDiscoverySnapshotAsyncEXT [%s]", openxr_string(result));
		return;
	}

	ctx->discovery_pending = true;
	uint64_t context_id = ctx->context_id;

	xr_ext_future_on_finish(future, [](void* user_data, XrFutureEXT future) {
		uint64_t context_id = (uint64_t)(uintptr_t)user_data;

		spatial_category_ category = spatial_category_max;
		xr_spatial_context_t* ctx = nullptr;
		for (int32_t i = 0; i < spatial_category_max; i++) {
			if (local.contexts[i].context_id == context_id) {
				category = (spatial_category_)i;
				ctx = &local.contexts[i];
				break;
			}
		}

		if (!ctx) return;
		ctx->discovery_pending = false;

		if (!ctx->enabled || !ctx->context_ready || ctx->context == XR_NULL_HANDLE)
			return;

		XrCreateSpatialDiscoverySnapshotCompletionInfoEXT completion_info = { XR_TYPE_CREATE_SPATIAL_DISCOVERY_SNAPSHOT_COMPLETION_INFO_EXT };
		completion_info.baseSpace = xr_app_space;
		completion_info.time      = xr_time;
		completion_info.future    = future;

		XrCreateSpatialDiscoverySnapshotCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_DISCOVERY_SNAPSHOT_COMPLETION_EXT };
		XrResult result = xrCreateSpatialDiscoverySnapshotCompleteEXT(ctx->context, &completion_info, &completion);
		if (XR_FAILED(result) || XR_FAILED(completion.futureResult)) {
			log_warnf("xrCreateSpatialDiscoverySnapshotCompleteEXT [%s]", openxr_string(XR_FAILED(result) ? result : completion.futureResult));
			return;
		}

		switch (category) {
		case spatial_category_marker: xr_spatial_marker_process_snapshot(completion.snapshot); break;
		case spatial_category_plane:  xr_spatial_plane_process_snapshot(completion.snapshot);  break;
		case spatial_category_anchor: xr_spatial_anchor_process_snapshot(completion.snapshot); break;
		default: break;
		}

		xrDestroySpatialSnapshotEXT(completion.snapshot);
	}, (void*)(uintptr_t)context_id);
}

static void xr_spatial_create_update_snapshot(spatial_category_ category) {
	xr_spatial_context_t* ctx = &local.contexts[category];
	if (!ctx->enabled || !ctx->context_ready || ctx->context == XR_NULL_HANDLE)
		return;

	array_t<XrSpatialEntityEXT> entities;
	switch (category) {
	case spatial_category_marker:
		for (int32_t i = 0; i < local.markers.count; i++)
			if (local.markers[i].entity_handle != XR_NULL_HANDLE)
				entities.add(local.markers[i].entity_handle);
		break;
	case spatial_category_plane:
		for (int32_t i = 0; i < local.planes.count; i++)
			if (local.planes[i].entity_handle != XR_NULL_HANDLE)
				entities.add(local.planes[i].entity_handle);
		break;
	case spatial_category_anchor:
		for (int32_t i = 0; i < local.anchors.count; i++)
			if (local.anchors[i].entity_handle != XR_NULL_HANDLE)
				entities.add(local.anchors[i].entity_handle);
		break;
	default:
		return;
	}

	if (entities.count == 0)
		return;

	XrSpatialUpdateSnapshotCreateInfoEXT create_info = { XR_TYPE_SPATIAL_UPDATE_SNAPSHOT_CREATE_INFO_EXT };
	create_info.entityCount = entities.count;
	create_info.entities    = entities.data;
	create_info.baseSpace   = xr_app_space;
	create_info.time        = xr_time;

	XrSpatialSnapshotEXT snapshot = XR_NULL_HANDLE;
	XrResult result = xrCreateSpatialUpdateSnapshotEXT(ctx->context, &create_info, &snapshot);
	entities.free();
	
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialUpdateSnapshotEXT [%s]", openxr_string(result));
		return;
	}

	switch (category) {
	case spatial_category_marker: xr_spatial_marker_process_snapshot(snapshot); break;
	case spatial_category_plane:  xr_spatial_plane_process_snapshot(snapshot);  break;
	case spatial_category_anchor: xr_spatial_anchor_process_snapshot(snapshot); break;
	default: break;
	}

	xrDestroySpatialSnapshotEXT(snapshot);
}

///////////////////////////////////////////

static bool xr_spatial_marker_create_context_ex(const world_marker_config_t* config, void (*callback)(void*, const world_marker_t*), void* callback_ctx) {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_marker];

	world_marker_types_ types = config ? config->types : world_marker_types_none;
	aruco_dict_ aruco_dict = config ? config->aruco_dict : aruco_dict_aruco4x4_50;
	apriltag_dict_ apriltag_dict = config ? config->apriltag_dict : apriltag_dict_tag36h11;
	if (ctx->context != XR_NULL_HANDLE) {
		local.marker_state.callback         = callback;
		local.marker_state.callback_context = callback_ctx;
		local.marker_state.enabled_types    = (world_marker_types_)(local.marker_state.enabled_types | types);
		return true;
	}
	const XrSpatialCapabilityConfigurationBaseHeaderEXT* configs[4];
	uint32_t config_count = 0;

	XrSpatialComponentTypeEXT components[] = {
		XR_SPATIAL_COMPONENT_TYPE_MARKER_EXT,
		XR_SPATIAL_COMPONENT_TYPE_BOUNDED_2D_EXT,
		XR_SPATIAL_COMPONENT_TYPE_PARENT_EXT,
	};
	const uint32_t component_count = (uint32_t)(sizeof(components) / sizeof(components[0]));
	XrSpatialCapabilityConfigurationQrCodeEXT      cfg_qr      = { XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_QR_CODE_EXT };
	XrSpatialCapabilityConfigurationMicroQrCodeEXT cfg_microqr = { XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_MICRO_QR_CODE_EXT };
	XrSpatialCapabilityConfigurationArucoMarkerEXT cfg_aruco   = { XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_ARUCO_MARKER_EXT };
	XrSpatialCapabilityConfigurationAprilTagEXT    cfg_april   = { XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_APRIL_TAG_EXT };
	if ((types & world_marker_types_qr) && (local.capabilities & world_tracking_caps_marker_qr)) {
		cfg_qr.capability            = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_QR_CODE_EXT;
		cfg_qr.enabledComponentCount = component_count;
		cfg_qr.enabledComponents     = components;
		configs[config_count++] = (XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_qr;
	}

	if ((types & world_marker_types_micro_qr) && (local.capabilities & world_tracking_caps_marker_micro_qr)) {
		cfg_microqr.capability            = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_MICRO_QR_CODE_EXT;
		cfg_microqr.enabledComponentCount = component_count;
		cfg_microqr.enabledComponents     = components;
		configs[config_count++] = (XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_microqr;
	}

	if ((types & world_marker_types_aruco) && (local.capabilities & world_tracking_caps_marker_aruco)) {
		cfg_aruco.capability            = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_ARUCO_MARKER_EXT;
		cfg_aruco.enabledComponentCount = component_count;
		cfg_aruco.enabledComponents     = components;
		cfg_aruco.arUcoDict             = xr_aruco_dict_to_xr(aruco_dict);
		configs[config_count++] = (XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_aruco;
	}

	if ((types & world_marker_types_april_tag) && (local.capabilities & world_tracking_caps_marker_april)) {
		cfg_april.capability            = XR_SPATIAL_CAPABILITY_MARKER_TRACKING_APRIL_TAG_EXT;
		cfg_april.enabledComponentCount = component_count;
		cfg_april.enabledComponents     = components;
		cfg_april.aprilDict             = xr_apriltag_dict_to_xr(apriltag_dict);
		configs[config_count++] = (XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_april;
	}

	if (config_count == 0)
		return false;

	local.marker_state.aruco_dict    = aruco_dict;
	local.marker_state.apriltag_dict = apriltag_dict;

	XrSpatialContextCreateInfoEXT create_info = { XR_TYPE_SPATIAL_CONTEXT_CREATE_INFO_EXT };
	create_info.capabilityConfigCount = config_count;
	create_info.capabilityConfigs     = configs;

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrCreateSpatialContextAsyncEXT(xr_session, &create_info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialContextAsyncEXT [%s]", openxr_string(result));
		return false;
	}

	ctx->enabled    = true;
	ctx->context_id = ++local.next_context_id;

	local.marker_state.enabled_types    = types;
	local.marker_state.callback         = callback;
	local.marker_state.callback_context = callback_ctx;

	uint64_t context_id = ctx->context_id;

	xr_ext_future_on_finish(future, [](void* user_data, XrFutureEXT future) {
		uint64_t context_id = (uint64_t)(uintptr_t)user_data;
		xr_spatial_context_t* ctx = &local.contexts[spatial_category_marker];

		// Context was stopped before completion
		if (ctx->context_id != context_id || !ctx->enabled) {
			XrCreateSpatialContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT };
			XrResult result = xrCreateSpatialContextCompleteEXT(xr_session, future, &completion);
			if (XR_SUCCEEDED(result) && completion.spatialContext != XR_NULL_HANDLE) {
				xrDestroySpatialContextEXT(completion.spatialContext);
			}
			return;
		}

		XrCreateSpatialContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT };
		XrResult result = xrCreateSpatialContextCompleteEXT(xr_session, future, &completion);
		if (XR_FAILED(result) || XR_FAILED(completion.futureResult)) {
			log_warnf("xrCreateSpatialContextCompleteEXT [%s]", openxr_string(XR_FAILED(result) ? result : completion.futureResult));
			ctx->enabled = false;
			return;
		}

		ctx->context       = completion.spatialContext;
		ctx->context_ready = true;
		log_infof("Spatial marker context ready, types: 0x%x", (unsigned int)local.marker_state.enabled_types);
	}, (void*)(uintptr_t)context_id);

	return true;
}

static void xr_spatial_marker_process_snapshot(XrSpatialSnapshotEXT snapshot) {
	if (snapshot == XR_NULL_HANDLE)
		return;

	for (int32_t i = 0; i < local.markers.count; i++) {
		local.markers[i].tracking_state = world_tracking_state_stopped;
	}

	XrSpatialComponentTypeEXT components[] = { XR_SPATIAL_COMPONENT_TYPE_MARKER_EXT };
	XrSpatialComponentDataQueryConditionEXT query_condition = { XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_CONDITION_EXT };
	query_condition.componentTypes     = components;
	query_condition.componentTypeCount = 1;

	XrSpatialComponentDataQueryResultEXT query_result = { XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_RESULT_EXT };
	XrResult result = xrQuerySpatialComponentDataEXT(snapshot, &query_condition, &query_result);
	if (XR_FAILED(result)) {
		log_warnf("xrQuerySpatialComponentDataEXT [%s]", openxr_string(result));
		return;
	}

	if (query_result.entityIdCountOutput == 0)
		return;

	// Allocate arrays for two-call idiom
	uint32_t count = query_result.entityIdCountOutput;
	XrSpatialEntityIdEXT*            entity_ids    = sk_malloc_t(XrSpatialEntityIdEXT, count);
	XrSpatialEntityTrackingStateEXT* entity_states = sk_malloc_t(XrSpatialEntityTrackingStateEXT, count);
	XrSpatialMarkerDataEXT*          marker_data   = sk_malloc_t(XrSpatialMarkerDataEXT, count);
	XrSpatialBounded2DDataEXT*       bounds_data   = sk_malloc_t(XrSpatialBounded2DDataEXT, count);
	XrSpatialEntityIdEXT*            parent_ids    = sk_malloc_t(XrSpatialEntityIdEXT, count);

	query_result.entityIds                = entity_ids;
	query_result.entityIdCapacityInput    = count;
	query_result.entityStates             = entity_states;
	query_result.entityStateCapacityInput = count;

	XrSpatialComponentMarkerListEXT marker_list = { XR_TYPE_SPATIAL_COMPONENT_MARKER_LIST_EXT };
	marker_list.markerCount = count;
	marker_list.markers     = marker_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&marker_list);

	XrSpatialComponentBounded2DListEXT bounds_list = { XR_TYPE_SPATIAL_COMPONENT_BOUNDED_2D_LIST_EXT };
	bounds_list.boundCount = count;
	bounds_list.bounds     = bounds_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&bounds_list);

	XrSpatialComponentParentListEXT parent_list = { XR_TYPE_SPATIAL_COMPONENT_PARENT_LIST_EXT };
	parent_list.parentCount = count;
	parent_list.parents     = parent_ids;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&parent_list);

	result = xrQuerySpatialComponentDataEXT(snapshot, &query_condition, &query_result);
	if (XR_FAILED(result)) {
		log_warnf("xrQuerySpatialComponentDataEXT second call [%s]", openxr_string(result));
		goto cleanup;
	}

	for (uint32_t i = 0; i < query_result.entityIdCountOutput; i++) {
		bool valid_type = false;
		world_marker_type_ marker_type = xr_capability_to_marker_type(marker_data[i].capability, &valid_type);
		if (!valid_type || !xr_marker_type_in_mask(marker_type, local.marker_state.enabled_types))
			continue;
		XrSpatialBufferGetInfoEXT buffer_info = { XR_TYPE_SPATIAL_BUFFER_GET_INFO_EXT };
		buffer_info.bufferId = marker_data[i].data.bufferId;

		uint32_t str_len = 0;
		result = xrGetSpatialBufferStringEXT(snapshot, &buffer_info, 0, &str_len, nullptr);
		if (XR_FAILED(result) || str_len == 0)
			continue;

		char* str_data = sk_malloc_t(char, str_len);
		result = xrGetSpatialBufferStringEXT(snapshot, &buffer_info, str_len, &str_len, str_data);
		if (XR_FAILED(result)) {
			sk_free(str_data);
			continue;
		}
		world_tracking_state_ tracking_state = xr_to_tracking_state(entity_states[i]);
		const XrPosef& xr_pose = bounds_data[i].center;
		uint64_t parent_id = (uint64_t)parent_ids[i];
		world_marker_t marker = {};
		marker.pose.position    = { xr_pose.position.x, xr_pose.position.y, xr_pose.position.z };
		marker.pose.orientation = { xr_pose.orientation.x, xr_pose.orientation.y, xr_pose.orientation.z, xr_pose.orientation.w };
		marker.size             = { bounds_data[i].extents.width, bounds_data[i].extents.height };
		marker.type             = marker_type;
		marker.tracking_state   = tracking_state;
		marker.id               = (uint64_t)entity_ids[i];
		marker.parent_id        = parent_id;
		marker.data             = str_data;
		bool found = false;
		for (int32_t j = 0; j < local.markers.count; j++) {
			if (local.markers[j].entity_id == marker.id) {
				sk_free(local.markers[j].data);
				local.markers[j].pose           = marker.pose;
				local.markers[j].size           = marker.size;
				local.markers[j].type           = marker_type;
				local.markers[j].tracking_state = tracking_state;
				local.markers[j].parent_id      = parent_id;
				local.markers[j].data           = str_data;
				found = true;
				break;
			}
		}
		if (!found) {
			xr_tracked_marker_t tracked = {};
			tracked.entity_id      = marker.id;
			tracked.entity_handle  = xr_entity_handle_from_id(spatial_category_marker, entity_ids[i]);
			tracked.parent_id      = parent_id;
			tracked.type           = marker_type;
			tracked.pose           = marker.pose;
			tracked.size           = marker.size;
			tracked.tracking_state = tracking_state;
			tracked.data           = str_data;
			local.markers.add(tracked);
		}
		if (local.marker_state.callback) {
			local.marker_state.callback(local.marker_state.callback_context, &marker);
		}
	}

cleanup:
	sk_free(entity_ids);
	sk_free(entity_states);
	sk_free(marker_data);
	sk_free(bounds_data);
	sk_free(parent_ids);
}

///////////////////////////////////////////

static bool xr_spatial_plane_create_context(void (*callback)(void*, const world_surface_t*), void* callback_ctx) {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_plane];

	if (ctx->context != XR_NULL_HANDLE) {
		local.plane_state.callback         = callback;
		local.plane_state.callback_context = callback_ctx;
		return true;
	}

	if (!(local.capabilities & world_tracking_caps_surface))
		return false;

	XrSpatialComponentTypeEXT components[] = {
		XR_SPATIAL_COMPONENT_TYPE_PLANE_ALIGNMENT_EXT,
		XR_SPATIAL_COMPONENT_TYPE_BOUNDED_2D_EXT,
		XR_SPATIAL_COMPONENT_TYPE_BOUNDED_3D_EXT,
		XR_SPATIAL_COMPONENT_TYPE_PLANE_SEMANTIC_LABEL_EXT,
		XR_SPATIAL_COMPONENT_TYPE_POLYGON_2D_EXT,
		XR_SPATIAL_COMPONENT_TYPE_MESH_2D_EXT,
		XR_SPATIAL_COMPONENT_TYPE_PARENT_EXT,
	};
	const uint32_t component_count = (uint32_t)(sizeof(components) / sizeof(components[0]));

	XrSpatialCapabilityConfigurationPlaneTrackingEXT cfg_plane = { XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_PLANE_TRACKING_EXT };
	cfg_plane.capability            = XR_SPATIAL_CAPABILITY_PLANE_TRACKING_EXT;
	cfg_plane.enabledComponentCount = component_count;
	cfg_plane.enabledComponents     = components;

	const XrSpatialCapabilityConfigurationBaseHeaderEXT* configs[] = {
		(XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_plane
	};

	XrSpatialContextCreateInfoEXT create_info = { XR_TYPE_SPATIAL_CONTEXT_CREATE_INFO_EXT };
	create_info.capabilityConfigCount = 1;
	create_info.capabilityConfigs     = configs;

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrCreateSpatialContextAsyncEXT(xr_session, &create_info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialContextAsyncEXT (plane) [%s]", openxr_string(result));
		return false;
	}

	ctx->enabled    = true;
	ctx->context_id = ++local.next_context_id;

	local.plane_state.callback         = callback;
	local.plane_state.callback_context = callback_ctx;

	uint64_t context_id = ctx->context_id;

	xr_ext_future_on_finish(future, [](void* user_data, XrFutureEXT future) {
		uint64_t context_id = (uint64_t)(uintptr_t)user_data;
		xr_spatial_context_t* ctx = &local.contexts[spatial_category_plane];

		if (ctx->context_id != context_id || !ctx->enabled) {
			XrCreateSpatialContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT };
			XrResult result = xrCreateSpatialContextCompleteEXT(xr_session, future, &completion);
			if (XR_SUCCEEDED(result) && completion.spatialContext != XR_NULL_HANDLE) {
				xrDestroySpatialContextEXT(completion.spatialContext);
			}
			return;
		}

		XrCreateSpatialContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT };
		XrResult result = xrCreateSpatialContextCompleteEXT(xr_session, future, &completion);
		if (XR_FAILED(result) || XR_FAILED(completion.futureResult)) {
			log_warnf("xrCreateSpatialContextCompleteEXT (plane) [%s]", openxr_string(XR_FAILED(result) ? result : completion.futureResult));
			ctx->enabled = false;
			return;
		}

		ctx->context       = completion.spatialContext;
		ctx->context_ready = true;
		log_info("Spatial plane tracking context ready");
	}, (void*)(uintptr_t)context_id);

	return true;
}

static void xr_spatial_plane_process_snapshot(XrSpatialSnapshotEXT snapshot) {
	if (snapshot == XR_NULL_HANDLE)
		return;

	for (int32_t i = 0; i < local.planes.count; i++) {
		local.planes[i].tracking_state = world_tracking_state_stopped;
	}

	XrSpatialComponentTypeEXT components[] = { XR_SPATIAL_COMPONENT_TYPE_PLANE_ALIGNMENT_EXT };
	XrSpatialComponentDataQueryConditionEXT query_condition = { XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_CONDITION_EXT };
	query_condition.componentTypes     = components;
	query_condition.componentTypeCount = 1;

	XrSpatialComponentDataQueryResultEXT query_result = { XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_RESULT_EXT };
	XrResult result = xrQuerySpatialComponentDataEXT(snapshot, &query_condition, &query_result);
	if (XR_FAILED(result)) {
		log_warnf("xrQuerySpatialComponentDataEXT (plane) [%s]", openxr_string(result));
		return;
	}

	if (query_result.entityIdCountOutput == 0)
		return;

	uint32_t count = query_result.entityIdCountOutput;
	XrSpatialEntityIdEXT*            entity_ids    = sk_malloc_t(XrSpatialEntityIdEXT, count);
	XrSpatialEntityTrackingStateEXT* entity_states = sk_malloc_t(XrSpatialEntityTrackingStateEXT, count);
	XrSpatialPlaneAlignmentEXT*      align_data    = sk_malloc_t(XrSpatialPlaneAlignmentEXT, count);
	XrSpatialBounded2DDataEXT*       bounds_data   = sk_malloc_t(XrSpatialBounded2DDataEXT, count);
	XrBoxf*                          bounds3d_data = sk_malloc_t(XrBoxf, count);
	XrSpatialPlaneSemanticLabelEXT*  label_data    = sk_malloc_t(XrSpatialPlaneSemanticLabelEXT, count);
	XrSpatialPolygon2DDataEXT*       polygon_data  = sk_malloc_t(XrSpatialPolygon2DDataEXT, count);
	XrSpatialMeshDataEXT*            mesh_data     = sk_malloc_t(XrSpatialMeshDataEXT, count);
	XrSpatialEntityIdEXT*            parent_ids    = sk_malloc_t(XrSpatialEntityIdEXT, count);

	query_result.entityIds                = entity_ids;
	query_result.entityIdCapacityInput    = count;
	query_result.entityStates             = entity_states;
	query_result.entityStateCapacityInput = count;

	XrSpatialComponentPlaneAlignmentListEXT align_list = { XR_TYPE_SPATIAL_COMPONENT_PLANE_ALIGNMENT_LIST_EXT };
	align_list.planeAlignmentCount = count;
	align_list.planeAlignments     = align_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&align_list);

	XrSpatialComponentBounded2DListEXT bounds_list = { XR_TYPE_SPATIAL_COMPONENT_BOUNDED_2D_LIST_EXT };
	bounds_list.boundCount = count;
	bounds_list.bounds     = bounds_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&bounds_list);

	XrSpatialComponentBounded3DListEXT bounds3d_list = { XR_TYPE_SPATIAL_COMPONENT_BOUNDED_3D_LIST_EXT };
	bounds3d_list.boundCount = count;
	bounds3d_list.bounds     = bounds3d_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&bounds3d_list);

	XrSpatialComponentPlaneSemanticLabelListEXT label_list = { XR_TYPE_SPATIAL_COMPONENT_PLANE_SEMANTIC_LABEL_LIST_EXT };
	label_list.semanticLabelCount = count;
	label_list.semanticLabels     = label_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&label_list);

	XrSpatialComponentPolygon2DListEXT polygon_list = { XR_TYPE_SPATIAL_COMPONENT_POLYGON_2D_LIST_EXT };
	polygon_list.polygonCount = count;
	polygon_list.polygons     = polygon_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&polygon_list);

	XrSpatialComponentMesh2DListEXT mesh_list = { XR_TYPE_SPATIAL_COMPONENT_MESH_2D_LIST_EXT };
	mesh_list.meshCount = count;
	mesh_list.meshes    = mesh_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&mesh_list);

	XrSpatialComponentParentListEXT parent_list = { XR_TYPE_SPATIAL_COMPONENT_PARENT_LIST_EXT };
	parent_list.parentCount = count;
	parent_list.parents     = parent_ids;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&parent_list);

	result = xrQuerySpatialComponentDataEXT(snapshot, &query_condition, &query_result);
	if (XR_FAILED(result)) {
		log_warnf("xrQuerySpatialComponentDataEXT (plane) second call [%s]", openxr_string(result));
		goto cleanup;
	}

	for (uint32_t i = 0; i < query_result.entityIdCountOutput; i++) {
		world_tracking_state_ tracking_state = xr_to_tracking_state(entity_states[i]);
		const XrPosef& xr_pose = bounds_data[i].center;

		xr_tracked_plane_t* tracked = nullptr;
		for (int32_t j = 0; j < local.planes.count; j++) {
			if (local.planes[j].entity_id == (uint64_t)entity_ids[i]) {
				tracked = &local.planes[j];
				break;
			}
		}
		if (!tracked) {
			xr_tracked_plane_t new_plane = {};
			new_plane.entity_id     = (uint64_t)entity_ids[i];
			new_plane.entity_handle = xr_entity_handle_from_id(spatial_category_plane, entity_ids[i]);
			local.planes.add(new_plane);
			tracked = &local.planes[local.planes.count - 1];
		}
		tracked->pose.position    = { xr_pose.position.x, xr_pose.position.y, xr_pose.position.z };
		tracked->pose.orientation = { xr_pose.orientation.x, xr_pose.orientation.y, xr_pose.orientation.z, xr_pose.orientation.w };
		tracked->bounds           = { bounds_data[i].extents.width, bounds_data[i].extents.height };
		tracked->alignment        = xr_alignment_to_plane_type(align_data[i]);
		tracked->label            = xr_semantic_to_plane_label(label_data[i]);
		tracked->tracking_state   = tracking_state;
		tracked->parent_id        = (uint64_t)parent_ids[i];

		const XrBoxf& box = bounds3d_data[i];
		tracked->bounds3d = { box.extents.width, box.extents.height, box.extents.depth };

		if (polygon_data[i].vertexBuffer.bufferId != 0) {
			xr_get_vec2_buffer(snapshot, polygon_data[i].vertexBuffer, &tracked->polygon);
		}

		if (mesh_data[i].vertexBuffer.bufferId != 0) {
			xr_get_vec2_buffer(snapshot, mesh_data[i].vertexBuffer, &tracked->mesh_vertices);
			if (mesh_data[i].indexBuffer.bufferId != 0) {
				xr_get_index_buffer(snapshot, mesh_data[i].indexBuffer, &tracked->mesh_indices);
			}
		}
		world_surface_t plane = {};
		plane.pose              = tracked->pose;
		plane.bounds            = tracked->bounds;
		plane.bounds3d          = tracked->bounds3d;
		plane.alignment         = tracked->alignment;
		plane.label             = tracked->label;
		plane.tracking_state    = tracking_state;
		plane.id                = tracked->entity_id;
		plane.parent_id         = tracked->parent_id;
		plane.polygon           = tracked->polygon.data;
		plane.polygon_count     = tracked->polygon.count;
		plane.mesh_vertices     = tracked->mesh_vertices.data;
		plane.mesh_vertex_count = tracked->mesh_vertices.count;
		plane.mesh_indices      = tracked->mesh_indices.data;
		plane.mesh_index_count  = tracked->mesh_indices.count;
		if (local.plane_state.callback) {
			local.plane_state.callback(local.plane_state.callback_context, &plane);
		}
	}

cleanup:
	sk_free(entity_ids);
	sk_free(entity_states);
	sk_free(align_data);
	sk_free(bounds_data);
	sk_free(bounds3d_data);
	sk_free(label_data);
	sk_free(polygon_data);
	sk_free(mesh_data);
	sk_free(parent_ids);
}

///////////////////////////////////////////

static bool xr_spatial_anchor_create_context(void (*callback)(void*, const world_anchor_t*), void* callback_ctx) {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_anchor];

	if (ctx->context != XR_NULL_HANDLE) {
		local.anchor_state.callback         = callback;
		local.anchor_state.callback_context = callback_ctx;
		return true;
	}

	if (!(local.capabilities & world_tracking_caps_anchor))
		return false;

	XrSpatialComponentTypeEXT components[] = {
		XR_SPATIAL_COMPONENT_TYPE_ANCHOR_EXT,
		XR_SPATIAL_COMPONENT_TYPE_BOUNDED_3D_EXT,
		XR_SPATIAL_COMPONENT_TYPE_PARENT_EXT,
		XR_SPATIAL_COMPONENT_TYPE_MESH_3D_EXT,
	};
	const uint32_t component_count = (uint32_t)(sizeof(components) / sizeof(components[0]));

	XrSpatialCapabilityConfigurationAnchorEXT cfg_anchor = { XR_TYPE_SPATIAL_CAPABILITY_CONFIGURATION_ANCHOR_EXT };
	cfg_anchor.capability            = XR_SPATIAL_CAPABILITY_ANCHOR_EXT;
	cfg_anchor.enabledComponentCount = component_count;
	cfg_anchor.enabledComponents     = components;

	const XrSpatialCapabilityConfigurationBaseHeaderEXT* configs[2];
	uint32_t config_count = 0;
	configs[config_count++] = (XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_anchor;

	XrSpatialContextPersistenceConfigEXT cfg_persistence = { XR_TYPE_SPATIAL_CONTEXT_PERSISTENCE_CONFIG_EXT };
	if (local.persistence_available && local.persistence_context != XR_NULL_HANDLE) {
		cfg_persistence.persistenceContextCount = 1;
		cfg_persistence.persistenceContexts     = &local.persistence_context;
		configs[config_count++] = (XrSpatialCapabilityConfigurationBaseHeaderEXT*)&cfg_persistence;
	}

	XrSpatialContextCreateInfoEXT create_info = { XR_TYPE_SPATIAL_CONTEXT_CREATE_INFO_EXT };
	create_info.capabilityConfigCount = config_count;
	create_info.capabilityConfigs     = configs;

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrCreateSpatialContextAsyncEXT(xr_session, &create_info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialContextAsyncEXT (anchor) [%s]", openxr_string(result));
		return false;
	}

	ctx->enabled    = true;
	ctx->context_id = ++local.next_context_id;

	local.anchor_state.callback         = callback;
	local.anchor_state.callback_context = callback_ctx;

	uint64_t context_id = ctx->context_id;

	xr_ext_future_on_finish(future, [](void* user_data, XrFutureEXT future) {
		uint64_t context_id = (uint64_t)(uintptr_t)user_data;
		xr_spatial_context_t* ctx = &local.contexts[spatial_category_anchor];

		if (ctx->context_id != context_id || !ctx->enabled) {
			XrCreateSpatialContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT };
			XrResult result = xrCreateSpatialContextCompleteEXT(xr_session, future, &completion);
			if (XR_SUCCEEDED(result) && completion.spatialContext != XR_NULL_HANDLE) {
				xrDestroySpatialContextEXT(completion.spatialContext);
			}
			return;
		}

		XrCreateSpatialContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_CONTEXT_COMPLETION_EXT };
		XrResult result = xrCreateSpatialContextCompleteEXT(xr_session, future, &completion);
		if (XR_FAILED(result) || XR_FAILED(completion.futureResult)) {
			log_warnf("xrCreateSpatialContextCompleteEXT (anchor) [%s]", openxr_string(XR_FAILED(result) ? result : completion.futureResult));
			ctx->enabled = false;
			return;
		}

		ctx->context       = completion.spatialContext;
		ctx->context_ready = true;
		log_info("Spatial anchor context ready");
	}, (void*)(uintptr_t)context_id);

	return true;
}

static void xr_spatial_anchor_process_snapshot(XrSpatialSnapshotEXT snapshot) {
	if (snapshot == XR_NULL_HANDLE)
		return;

	for (int32_t i = 0; i < local.anchors.count; i++) {
		local.anchors[i].tracking_state = world_tracking_state_stopped;
	}

	XrSpatialComponentTypeEXT components[] = { XR_SPATIAL_COMPONENT_TYPE_ANCHOR_EXT };
	XrSpatialComponentDataQueryConditionEXT query_condition = { XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_CONDITION_EXT };
	query_condition.componentTypes     = components;
	query_condition.componentTypeCount = 1;

	XrSpatialComponentDataQueryResultEXT query_result = { XR_TYPE_SPATIAL_COMPONENT_DATA_QUERY_RESULT_EXT };
	XrResult result = xrQuerySpatialComponentDataEXT(snapshot, &query_condition, &query_result);
	if (XR_FAILED(result)) {
		log_warnf("xrQuerySpatialComponentDataEXT (anchor) [%s]", openxr_string(result));
		return;
	}

	if (query_result.entityIdCountOutput == 0)
		return;

	uint32_t count = query_result.entityIdCountOutput;
	XrSpatialEntityIdEXT*            entity_ids    = sk_malloc_t(XrSpatialEntityIdEXT, count);
	XrSpatialEntityTrackingStateEXT* entity_states = sk_malloc_t(XrSpatialEntityTrackingStateEXT, count);
	XrPosef*                         pose_data     = sk_malloc_t(XrPosef, count);
	XrSpatialPersistenceDataEXT*     persist_data  = sk_malloc_t(XrSpatialPersistenceDataEXT, count);
	XrBoxf*                          bounds3d_data = sk_malloc_t(XrBoxf, count);
	XrSpatialEntityIdEXT*            parent_ids    = sk_malloc_t(XrSpatialEntityIdEXT, count);
	XrSpatialMeshDataEXT*            mesh3d_data   = sk_malloc_t(XrSpatialMeshDataEXT, count);

	query_result.entityIds                = entity_ids;
	query_result.entityIdCapacityInput    = count;
	query_result.entityStates             = entity_states;
	query_result.entityStateCapacityInput = count;

	XrSpatialComponentAnchorListEXT anchor_list = { XR_TYPE_SPATIAL_COMPONENT_ANCHOR_LIST_EXT };
	anchor_list.locationCount = count;
	anchor_list.locations     = pose_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&anchor_list);

	XrSpatialComponentPersistenceListEXT persist_list = { XR_TYPE_SPATIAL_COMPONENT_PERSISTENCE_LIST_EXT };
	persist_list.persistDataCount = count;
	persist_list.persistData      = persist_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&persist_list);

	XrSpatialComponentBounded3DListEXT bounds3d_list = { XR_TYPE_SPATIAL_COMPONENT_BOUNDED_3D_LIST_EXT };
	bounds3d_list.boundCount = count;
	bounds3d_list.bounds     = bounds3d_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&bounds3d_list);

	XrSpatialComponentParentListEXT parent_list = { XR_TYPE_SPATIAL_COMPONENT_PARENT_LIST_EXT };
	parent_list.parentCount = count;
	parent_list.parents     = parent_ids;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&parent_list);

	XrSpatialComponentMesh3DListEXT mesh3d_list = { XR_TYPE_SPATIAL_COMPONENT_MESH_3D_LIST_EXT };
	mesh3d_list.meshCount = count;
	mesh3d_list.meshes    = mesh3d_data;
	xr_insert_next((XrBaseHeader*)&query_result, (XrBaseHeader*)&mesh3d_list);

	result = xrQuerySpatialComponentDataEXT(snapshot, &query_condition, &query_result);
	if (XR_FAILED(result)) {
		log_warnf("xrQuerySpatialComponentDataEXT (anchor) second call [%s]", openxr_string(result));
		goto cleanup;
	}

	for (uint32_t i = 0; i < query_result.entityIdCountOutput; i++) {
		world_tracking_state_ tracking_state = xr_to_tracking_state(entity_states[i]);
		const XrPosef& xr_pose = pose_data[i];

		xr_tracked_anchor_t* tracked = nullptr;
		for (int32_t j = 0; j < local.anchors.count; j++) {
			if (local.anchors[j].entity_id == (uint64_t)entity_ids[i]) {
				tracked = &local.anchors[j];
				break;
			}
		}
		if (!tracked) {
			xr_tracked_anchor_t new_anchor = {};
			new_anchor.entity_id     = (uint64_t)entity_ids[i];
			new_anchor.entity_handle = xr_entity_handle_from_id(spatial_category_anchor, entity_ids[i]);
			local.anchors.add(new_anchor);
			tracked = &local.anchors[local.anchors.count - 1];
		}
		tracked->pose.position    = { xr_pose.position.x, xr_pose.position.y, xr_pose.position.z };
		tracked->pose.orientation = { xr_pose.orientation.x, xr_pose.orientation.y, xr_pose.orientation.z, xr_pose.orientation.w };
		tracked->tracking_state   = tracking_state;
		tracked->is_persistent    = persist_data[i].persistState == XR_SPATIAL_PERSISTENCE_STATE_LOADED_EXT;
		tracked->parent_id        = (uint64_t)parent_ids[i];

		const XrBoxf& box = bounds3d_data[i];
		tracked->bounds3d = { box.extents.width, box.extents.height, box.extents.depth };

		if (mesh3d_data[i].vertexBuffer.bufferId != 0) {
			xr_get_vec3_buffer(snapshot, mesh3d_data[i].vertexBuffer, &tracked->mesh_vertices);
			if (mesh3d_data[i].indexBuffer.bufferId != 0) {
				xr_get_index_buffer(snapshot, mesh3d_data[i].indexBuffer, &tracked->mesh_indices);
			}
		}

		if (tracked->is_persistent && persist_data[i].persistUuid.data[0] != 0) {
			const uint8_t* u = persist_data[i].persistUuid.data;
			snprintf(tracked->uuid, sizeof(tracked->uuid),
				"%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
				u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
				u[8], u[9], u[10], u[11], u[12], u[13], u[14], u[15]);
		} else {
			tracked->uuid[0] = '\0';
		}
		world_anchor_t anchor = {};
		xr_fill_anchor(*tracked, &anchor);
		if (local.anchor_state.callback) {
			local.anchor_state.callback(local.anchor_state.callback_context, &anchor);
		}
	}

cleanup:
	sk_free(entity_ids);
	sk_free(entity_states);
	sk_free(pose_data);
	sk_free(persist_data);
	sk_free(bounds3d_data);
	sk_free(parent_ids);
	sk_free(mesh3d_data);
}

static bool xr_spatial_create_persistence_context() {
	if (!local.persistence_available)
		return false;

	XrSpatialPersistenceContextCreateInfoEXT create_info = { XR_TYPE_SPATIAL_PERSISTENCE_CONTEXT_CREATE_INFO_EXT };

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrCreateSpatialPersistenceContextAsyncEXT(xr_session, &create_info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialPersistenceContextAsyncEXT [%s]", openxr_string(result));
		return false;
	}

	xr_ext_future_on_finish(future, [](void*, XrFutureEXT future) {
		XrCreateSpatialPersistenceContextCompletionEXT completion = { XR_TYPE_CREATE_SPATIAL_PERSISTENCE_CONTEXT_COMPLETION_EXT };
		XrResult result = xrCreateSpatialPersistenceContextCompleteEXT(xr_session, future, &completion);
		if (XR_FAILED(result) || XR_FAILED(completion.futureResult)) {
			log_warnf("xrCreateSpatialPersistenceContextCompleteEXT [%s]", openxr_string(XR_FAILED(result) ? result : completion.futureResult));
			local.persistence_available = false;
			return;
		}

		local.persistence_context       = completion.persistenceContext;
		local.persistence_context_ready = true;
		log_info("Spatial persistence context ready");
	}, nullptr);

	return true;
}

///////////////////////////////////////////

static void xr_fill_marker(const xr_tracked_marker_t& m, world_marker_t* out) {
	out->pose           = m.pose;
	out->size           = m.size;
	out->type           = m.type;
	out->tracking_state = m.tracking_state;
	out->id             = m.entity_id;
	out->parent_id      = m.parent_id;
	out->data           = m.data;
}

static void xr_fill_surface(const xr_tracked_plane_t& p, world_surface_t* out) {
	out->pose              = p.pose;
	out->bounds            = p.bounds;
	out->bounds3d          = p.bounds3d;
	out->alignment         = p.alignment;
	out->label             = p.label;
	out->tracking_state    = p.tracking_state;
	out->id                = p.entity_id;
	out->parent_id         = p.parent_id;
	out->polygon           = p.polygon.data;
	out->polygon_count     = p.polygon.count;
	out->mesh_vertices     = p.mesh_vertices.data;
	out->mesh_vertex_count = p.mesh_vertices.count;
	out->mesh_indices      = p.mesh_indices.data;
	out->mesh_index_count  = p.mesh_indices.count;
}

static void xr_fill_anchor(const xr_tracked_anchor_t& a, world_anchor_t* out) {
	out->pose              = a.pose;
	out->bounds3d          = a.bounds3d;
	out->tracking_state    = a.tracking_state;
	out->id                = a.entity_id;
	out->parent_id         = a.parent_id;
	out->is_persistent     = a.is_persistent ? 1 : 0;
	out->uuid              = a.uuid[0] ? a.uuid : nullptr;
	out->mesh_vertices     = a.mesh_vertices.data;
	out->mesh_vertex_count = a.mesh_vertices.count;
	out->mesh_indices      = a.mesh_indices.data;
	out->mesh_index_count  = a.mesh_indices.count;
}

bool32_t world_tracking_is_available() {
	return local.available ? 1 : 0;
}

world_tracking_caps_ world_tracking_get_capabilities() {
	return local.capabilities;
}

bool32_t world_tracking_has_marker_type(world_marker_type_ type) {
	switch (type) {
	case world_marker_type_qr_code:       return (local.capabilities & world_tracking_caps_marker_qr)       != 0;
	case world_marker_type_micro_qr_code: return (local.capabilities & world_tracking_caps_marker_micro_qr) != 0;
	case world_marker_type_aruco:         return (local.capabilities & world_tracking_caps_marker_aruco)    != 0;
	case world_marker_type_april_tag:     return (local.capabilities & world_tracking_caps_marker_april)    != 0;
	default: return false;
	}
}

///////////////////////////////////////////

bool32_t world_marker_start(world_marker_types_ types, void (*on_marker_event)(void* context, const world_marker_t* marker), void* context) {
	if (!local.available)
		return false;
	world_tracking_caps_ supported = xr_marker_types_to_caps(types);
	if ((supported & local.capabilities) == 0)
		return false;

	world_marker_config_t config = {};
	config.types        = types;
	config.aruco_dict   = aruco_dict_aruco4x4_50;
	config.apriltag_dict = apriltag_dict_tag36h11;
	return xr_spatial_marker_create_context_ex(&config, on_marker_event, context) ? 1 : 0;
}

bool32_t world_marker_start_ex(const world_marker_config_t* config, void (*on_event)(void* context, const world_marker_t* marker), void* context) {
	if (!local.available || !config)
		return false;
	world_tracking_caps_ supported = xr_marker_types_to_caps(config->types);
	if ((supported & local.capabilities) == 0)
		return false;
	return xr_spatial_marker_create_context_ex(config, on_event, context) ? 1 : 0;
}

void world_marker_stop() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_marker];

	// If discovery in-flight, mark for cleanup when complete
	if (ctx->discovery_pending) {
		ctx->enabled = false;
		local.marker_state.callback = nullptr;
		local.marker_state.callback_context = nullptr;
		return;
	}

	if (ctx->context != XR_NULL_HANDLE) {
		xrDestroySpatialContextEXT(ctx->context);
		ctx->context = XR_NULL_HANDLE;
	}

	ctx->context_ready = false;
	ctx->enabled       = false;
	for (int32_t i = 0; i < local.markers.count; i++) {
		sk_free(local.markers[i].data);
	}
	local.markers.clear();

	local.marker_state.enabled_types    = world_marker_types_none;
	local.marker_state.callback         = nullptr;
	local.marker_state.callback_context = nullptr;
}

bool32_t world_marker_is_active() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_marker];
	return ctx->enabled && ctx->context_ready ? 1 : 0;
}

world_marker_types_ world_marker_get_active_types() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_marker];
	if (!ctx->enabled || !ctx->context_ready)
		return world_marker_types_none;
	return local.marker_state.enabled_types;
}

int32_t world_marker_get_count() {
	return local.markers.count;
}

int32_t world_marker_get_count_of_type(world_marker_type_ type) {
	int32_t count = 0;
	for (int32_t i = 0; i < local.markers.count; i++) {
		if (local.markers[i].type == type)
			count++;
	}
	return count;
}

bool32_t world_marker_get_at(int32_t index, world_marker_t* out_marker) {
	if (!out_marker || index < 0 || index >= local.markers.count)
		return false;

	const xr_tracked_marker_t& m = local.markers[index];
	xr_fill_marker(m, out_marker);
	return true;
}

bool32_t world_marker_find(const char* data_utf8, world_marker_t* out_marker) {
	if (!out_marker || !data_utf8)
		return false;

	for (int32_t i = 0; i < local.markers.count; i++) {
		if (local.markers[i].data && strcmp(local.markers[i].data, data_utf8) == 0) {
			xr_fill_marker(local.markers[i], out_marker);
			return true;
		}
	}
	return false;
}

bool32_t world_marker_find_by_id(uint64_t id, world_marker_t* out_marker) {
	if (!out_marker)
		return false;

	for (int32_t i = 0; i < local.markers.count; i++) {
		if (local.markers[i].entity_id == id) {
			xr_fill_marker(local.markers[i], out_marker);
			return true;
		}
	}
	return false;
}

void world_marker_refresh() {
	if (local.contexts[spatial_category_marker].enabled)
		xr_spatial_context_begin_discovery(spatial_category_marker);
}

///////////////////////////////////////////

bool32_t world_surface_start(void (*on_surface_event)(void* context, const world_surface_t* plane), void* context) {
	if (!local.available)
		return false;

	if (!(local.capabilities & world_tracking_caps_surface))
		return false;

	return xr_spatial_plane_create_context(on_surface_event, context) ? 1 : 0;
}

void world_surface_stop() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_plane];

	if (ctx->discovery_pending) {
		ctx->enabled = false;
		local.plane_state.callback = nullptr;
		local.plane_state.callback_context = nullptr;
		return;
	}

	if (ctx->context != XR_NULL_HANDLE) {
		xrDestroySpatialContextEXT(ctx->context);
		ctx->context = XR_NULL_HANDLE;
	}

	ctx->context_ready = false;
	ctx->enabled       = false;

	for (int32_t i = 0; i < local.planes.count; i++) {
		local.planes[i].polygon.free();
		local.planes[i].mesh_vertices.free();
		local.planes[i].mesh_indices.free();
	}
	local.planes.clear();

	local.plane_state.callback         = nullptr;
	local.plane_state.callback_context = nullptr;
}

bool32_t world_surface_is_active() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_plane];
	return ctx->enabled && ctx->context_ready ? 1 : 0;
}

int32_t world_surface_get_count() {
	return local.planes.count;
}

bool32_t world_surface_get_at(int32_t index, world_surface_t* out_plane) {
	if (!out_plane || index < 0 || index >= local.planes.count)
		return false;

	const xr_tracked_plane_t& p = local.planes[index];
	xr_fill_surface(p, out_plane);
	return true;
}

bool32_t world_surface_find_by_id(uint64_t id, world_surface_t* out_plane) {
	if (!out_plane)
		return false;

	for (int32_t i = 0; i < local.planes.count; i++) {
		if (local.planes[i].entity_id == id) {
			xr_fill_surface(local.planes[i], out_plane);
			return true;
		}
	}
	return false;
}

void world_surface_refresh() {
	if (local.contexts[spatial_category_plane].enabled)
		xr_spatial_context_begin_discovery(spatial_category_plane);
}

///////////////////////////////////////////

bool32_t world_anchor_start(void (*on_anchor_event)(void* context, const world_anchor_t* anchor), void* context) {
	if (!local.available)
		return false;

	if (!(local.capabilities & world_tracking_caps_anchor))
		return false;

	return xr_spatial_anchor_create_context(on_anchor_event, context) ? 1 : 0;
}

void world_anchor_stop() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_anchor];

	if (ctx->discovery_pending) {
		ctx->enabled = false;
		local.anchor_state.callback = nullptr;
		local.anchor_state.callback_context = nullptr;
		return;
	}

	if (ctx->context != XR_NULL_HANDLE) {
		xrDestroySpatialContextEXT(ctx->context);
		ctx->context = XR_NULL_HANDLE;
	}

	ctx->context_ready = false;
	ctx->enabled       = false;

	for (int32_t i = 0; i < local.anchors.count; i++) {
		local.anchors[i].mesh_vertices.free();
		local.anchors[i].mesh_indices.free();
	}
	local.anchors.clear();

	local.anchor_state.callback         = nullptr;
	local.anchor_state.callback_context = nullptr;
}

bool32_t world_anchor_is_active() {
	xr_spatial_context_t* ctx = &local.contexts[spatial_category_anchor];
	return ctx->enabled && ctx->context_ready ? 1 : 0;
}

int32_t world_anchor_get_count() {
	return local.anchors.count;
}

bool32_t world_anchor_get_at(int32_t index, world_anchor_t* out_anchor) {
	if (!out_anchor || index < 0 || index >= local.anchors.count)
		return false;

	xr_fill_anchor(local.anchors[index], out_anchor);
	return true;
}

bool32_t world_anchor_find_by_id(uint64_t id, world_anchor_t* out_anchor) {
	if (!out_anchor)
		return false;

	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].entity_id == id) {
			xr_fill_anchor(local.anchors[i], out_anchor);
			return true;
		}
	}
	return false;
}

bool32_t world_anchor_find_by_uuid(const char* uuid, world_anchor_t* out_anchor) {
	if (!out_anchor || !uuid)
		return false;

	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].uuid[0] && strcmp(local.anchors[i].uuid, uuid) == 0) {
			xr_fill_anchor(local.anchors[i], out_anchor);
			return true;
		}
	}
	return false;
}

bool32_t world_anchor_create(pose_t pose, void (*on_created)(void* context, const world_anchor_t* anchor), void* context) {
	if (!local.available)
		return false;

	xr_spatial_context_t* ctx = &local.contexts[spatial_category_anchor];
	if (!ctx->context_ready)
		return false;

	XrPosef xr_pose = {};
	xr_pose.position    = { pose.position.x, pose.position.y, pose.position.z };
	xr_pose.orientation = { pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w };

	XrSpatialAnchorCreateInfoEXT create_info = { XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_EXT };
	create_info.baseSpace = xr_app_space;
	create_info.time      = xr_time;
	create_info.pose      = xr_pose;

	XrSpatialEntityIdEXT entity_id  = {};
	XrSpatialEntityEXT   entity     = XR_NULL_HANDLE;
	XrResult result = xrCreateSpatialAnchorEXT(ctx->context, &create_info, &entity_id, &entity);
	if (XR_FAILED(result)) {
		log_warnf("xrCreateSpatialAnchorEXT [%s]", openxr_string(result));
		if (on_created) on_created(context, nullptr);
		return false;
	}

	xr_tracked_anchor_t tracked = {};
	tracked.entity_id      = (uint64_t)entity_id;
	tracked.entity_handle  = entity;
	tracked.pose           = pose;
	tracked.tracking_state = world_tracking_state_tracking;
	tracked.is_persistent  = false;
	tracked.uuid[0]        = '\0';
	local.anchors.add(tracked);

	world_anchor_t anchor = {};
	anchor.pose           = pose;
	anchor.tracking_state = world_tracking_state_tracking;
	anchor.id             = (uint64_t)entity_id;
	anchor.is_persistent  = 0;
	anchor.uuid           = nullptr;

	if (on_created) on_created(context, &anchor);
	return true;
}

bool32_t world_anchor_remove(uint64_t anchor_id) {
	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].entity_id == anchor_id) {
			if (local.anchors[i].entity_handle != XR_NULL_HANDLE) {
				xrDestroySpatialEntityEXT(local.anchors[i].entity_handle);
			}
			local.anchors.remove(i);
			return true;
		}
	}
	return false;
}

void world_anchor_refresh() {
	if (local.contexts[spatial_category_anchor].enabled)
		xr_spatial_context_begin_discovery(spatial_category_anchor);
}

///////////////////////////////////////////

bool32_t world_persistence_is_available() {
	return local.persistence_available && local.persistence_context_ready ? 1 : 0;
}

bool32_t world_anchor_persist(uint64_t anchor_id, void (*on_completed)(void* context, bool32_t success, const char* uuid), void* context) {
	if (!local.persistence_available || !local.persistence_context_ready)
		return false;

	xr_tracked_anchor_t* anchor = nullptr;
	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].entity_id == anchor_id) {
			anchor = &local.anchors[i];
			break;
		}
	}
	if (!anchor || anchor->entity_handle == XR_NULL_HANDLE)
		return false;

	XrSpatialEntityPersistInfoEXT persist_info = { XR_TYPE_SPATIAL_ENTITY_PERSIST_INFO_EXT };
	persist_info.spatialContext  = local.contexts[spatial_category_anchor].context;
	persist_info.spatialEntityId = (XrSpatialEntityIdEXT)anchor->entity_id;

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrPersistSpatialEntityAsyncEXT(local.persistence_context, &persist_info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrPersistSpatialEntityAsyncEXT [%s]", openxr_string(result));
		return false;
	}

	xr_persist_request_t request = {};
	request.anchor_id         = anchor_id;
	request.callback_persist  = on_completed;
	request.callback_context  = context;
	local.persist_requests.add(request);

	uint64_t req_anchor_id = anchor_id;

	xr_ext_future_on_finish(future, [](void* user_data, XrFutureEXT future) {
		uint64_t anchor_id = (uint64_t)(uintptr_t)user_data;

		xr_persist_request_t* request = nullptr;
		int32_t request_idx = -1;
		for (int32_t i = 0; i < local.persist_requests.count; i++) {
			if (local.persist_requests[i].anchor_id == anchor_id && local.persist_requests[i].callback_persist) {
				request = &local.persist_requests[i];
				request_idx = i;
				break;
			}
		}

		XrPersistSpatialEntityCompletionEXT completion = { XR_TYPE_PERSIST_SPATIAL_ENTITY_COMPLETION_EXT };
		XrResult result = xrPersistSpatialEntityCompleteEXT(local.persistence_context, future, &completion);

		bool success = XR_SUCCEEDED(result) && XR_SUCCEEDED(completion.futureResult);
		char uuid_str[37] = {};

		if (success) {
			for (int32_t i = 0; i < local.anchors.count; i++) {
				if (local.anchors[i].entity_id == anchor_id) {
					local.anchors[i].is_persistent = true;
					const uint8_t* u = completion.persistUuid.data;
					snprintf(local.anchors[i].uuid, sizeof(local.anchors[i].uuid),
						"%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
						u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7],
						u[8], u[9], u[10], u[11], u[12], u[13], u[14], u[15]);
					memcpy(uuid_str, local.anchors[i].uuid, sizeof(uuid_str));
					break;
				}
			}
		}

		if (request && request->callback_persist) {
			request->callback_persist(request->callback_context, success ? 1 : 0, success ? uuid_str : nullptr);
		}

		if (request_idx >= 0) {
			local.persist_requests.remove(request_idx);
		}
	}, (void*)(uintptr_t)req_anchor_id);

	return true;
}

bool32_t world_anchor_unpersist(uint64_t anchor_id, void (*on_completed)(void* context, bool32_t success), void* context) {
	if (!local.persistence_available || !local.persistence_context_ready)
		return false;

	xr_tracked_anchor_t* anchor = nullptr;
	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].entity_id == anchor_id) {
			anchor = &local.anchors[i];
			break;
		}
	}
	if (!anchor || anchor->entity_handle == XR_NULL_HANDLE || !anchor->is_persistent)
		return false;

	XrUuid uuid = {};
	if (anchor->uuid[0]) {
		unsigned int u[16];
		sscanf(anchor->uuid,
			"%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
			&u[0], &u[1], &u[2], &u[3], &u[4], &u[5], &u[6], &u[7],
			&u[8], &u[9], &u[10], &u[11], &u[12], &u[13], &u[14], &u[15]);
		for (int i = 0; i < 16; i++) uuid.data[i] = (uint8_t)u[i];
	}

	XrSpatialEntityUnpersistInfoEXT unpersist_info = { XR_TYPE_SPATIAL_ENTITY_UNPERSIST_INFO_EXT };
	unpersist_info.persistUuid = uuid;

	XrFutureEXT future = XR_NULL_FUTURE_EXT;
	XrResult result = xrUnpersistSpatialEntityAsyncEXT(local.persistence_context, &unpersist_info, &future);
	if (XR_FAILED(result)) {
		log_warnf("xrUnpersistSpatialEntityAsyncEXT [%s]", openxr_string(result));
		return false;
	}

	xr_persist_request_t request = {};
	request.anchor_id           = anchor_id;
	request.callback_unpersist  = on_completed;
	request.callback_context    = context;
	local.persist_requests.add(request);

	uint64_t req_anchor_id = anchor_id;

	xr_ext_future_on_finish(future, [](void* user_data, XrFutureEXT future) {
		uint64_t anchor_id = (uint64_t)(uintptr_t)user_data;

		xr_persist_request_t* request = nullptr;
		int32_t request_idx = -1;
		for (int32_t i = 0; i < local.persist_requests.count; i++) {
			if (local.persist_requests[i].anchor_id == anchor_id && local.persist_requests[i].callback_unpersist) {
				request = &local.persist_requests[i];
				request_idx = i;
				break;
			}
		}

		XrUnpersistSpatialEntityCompletionEXT completion = { XR_TYPE_UNPERSIST_SPATIAL_ENTITY_COMPLETION_EXT };
		XrResult result = xrUnpersistSpatialEntityCompleteEXT(local.persistence_context, future, &completion);

		bool success = XR_SUCCEEDED(result) && XR_SUCCEEDED(completion.futureResult);

		if (success) {
			for (int32_t i = 0; i < local.anchors.count; i++) {
				if (local.anchors[i].entity_id == anchor_id) {
					local.anchors[i].is_persistent = false;
					local.anchors[i].uuid[0] = '\0';
					break;
				}
			}
		}

		if (request && request->callback_unpersist) {
			request->callback_unpersist(request->callback_context, success ? 1 : 0);
		}

		if (request_idx >= 0) {
			local.persist_requests.remove(request_idx);
		}
	}, (void*)(uintptr_t)req_anchor_id);

	return true;
}

///////////////////////////////////////////

// Maps world_tracking_state_ to button_state_ for anchor_t compatibility
static button_state_ tracking_state_to_button(world_tracking_state_ state) {
	switch (state) {
	case world_tracking_state_tracking: return button_state_active;
	case world_tracking_state_paused:   return button_state_inactive;
	case world_tracking_state_stopped:  return button_state_inactive;
	default: return button_state_inactive;
	}
}

// Internal struct to track anchor_t <-> spatial entity mapping
typedef struct xr_anchor_bridge_t {
	anchor_t  anchor;
	uint64_t  entity_id;
	bool      pending_create;
} xr_anchor_bridge_t;

static array_t<xr_anchor_bridge_t> anchor_bridges = {};

bool xr_ext_spatial_anchors_available() {
	return local.available && (local.capabilities & world_tracking_caps_anchor) != 0;
}

anchor_t xr_ext_spatial_anchors_create(pose_t pose, const char* name_utf8) {
	if (!xr_ext_spatial_anchors_available())
		return nullptr;

	xr_spatial_context_t* ctx = &local.contexts[spatial_category_anchor];
	if (!ctx->context_ready) {
		if (!xr_spatial_anchor_create_context(nullptr, nullptr))
			return nullptr;
	}

	anchor_t anchor = anchor_create_manual(anchor_system_openxr_ext, pose, name_utf8, nullptr);
	anchor->tracked = button_state_inactive;

	xr_anchor_bridge_t bridge = {};
	bridge.anchor         = anchor;
	bridge.entity_id      = 0;
	bridge.pending_create = true;
	anchor_bridges.add(bridge);

	int32_t bridge_idx = anchor_bridges.count - 1;

	world_anchor_create(pose, [](void* context, const world_anchor_t* created_anchor) {
		int32_t idx = (int32_t)(intptr_t)context;
		if (idx < 0 || idx >= anchor_bridges.count)
			return;

		xr_anchor_bridge_t* bridge = &anchor_bridges[idx];
		if (!bridge->anchor || !created_anchor) {
			bridge->pending_create = false;
			return;
		}

		// Link the anchor_t to the spatial entity
		bridge->entity_id      = created_anchor->id;
		bridge->pending_create = false;
		bridge->anchor->data   = (void*)(uintptr_t)created_anchor->id;
		bridge->anchor->pose   = created_anchor->pose;
		bridge->anchor->tracked = tracking_state_to_button(created_anchor->tracking_state);

		anchor_mark_dirty(bridge->anchor);
	}, (void*)(intptr_t)bridge_idx);

	return anchor;
}

anchor_t xr_ext_spatial_anchors_from_tracked(uint64_t entity_id) {
	if (entity_id == 0)
		return nullptr;

	for (int32_t i = 0; i < anchor_bridges.count; i++) {
		if (anchor_bridges[i].entity_id == entity_id && anchor_bridges[i].anchor) {
			anchor_addref(anchor_bridges[i].anchor);
			return anchor_bridges[i].anchor;
		}
	}

	int32_t tracked_idx = -1;
	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].entity_id == entity_id) {
			tracked_idx = i;
			break;
		}
	}
	if (tracked_idx < 0)
		return nullptr;

	const xr_tracked_anchor_t& tracked = local.anchors[tracked_idx];

	// Use uuid as name if available, otherwise generate one
	const char* name  = tracked.uuid[0] != '\0' ? tracked.uuid : nullptr;
	anchor_t anchor   = anchor_create_manual(anchor_system_openxr_ext, tracked.pose, name, (void*)(uintptr_t)entity_id);
	anchor->tracked   = tracking_state_to_button(tracked.tracking_state);
	anchor->persisted = tracked.is_persistent;

	xr_anchor_bridge_t bridge = {};
	bridge.anchor         = anchor;
	bridge.entity_id      = entity_id;
	bridge.pending_create = false;
	anchor_bridges.add(bridge);

	return anchor;
}

void xr_ext_spatial_anchors_destroy(anchor_t anchor) {
	if (!anchor)
		return;

	uint64_t entity_id = (uint64_t)(uintptr_t)anchor->data;

	for (int32_t i = 0; i < anchor_bridges.count; i++) {
		if (anchor_bridges[i].anchor == anchor) {
			anchor_bridges.remove(i);
			break;
		}
	}

	if (entity_id != 0) {
		world_anchor_remove(entity_id);
	}
}

void xr_ext_spatial_anchors_clear_stored() {
	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].is_persistent) {
			world_anchor_unpersist(local.anchors[i].entity_id, nullptr, nullptr);
		}
	}
}

bool32_t xr_ext_spatial_anchors_persist(anchor_t anchor, bool32_t persist) {
	if (!anchor || !local.persistence_available)
		return false;

	uint64_t entity_id = (uint64_t)(uintptr_t)anchor->data;
	if (entity_id == 0)
		return false;

	if (persist) {
		return world_anchor_persist(entity_id, [](void* ctx, bool32_t success, const char* uuid) {
			anchor_t a = (anchor_t)ctx;
			if (a && success) {
				a->persisted = true;
				anchor_mark_dirty(a);
			}
		}, anchor);
	} else {
		return world_anchor_unpersist(entity_id, [](void* ctx, bool32_t success) {
			anchor_t a = (anchor_t)ctx;
			if (a && success) {
				a->persisted = false;
				anchor_mark_dirty(a);
			}
		}, anchor);
	}
}

anchor_caps_ xr_ext_spatial_anchors_capabilities() {
	anchor_caps_ caps = (anchor_caps_)0;

	if (local.persistence_available)
		caps = (anchor_caps_)(caps | anchor_caps_storable);

	// EXT spatial entities provide stability through the spatial context
	caps = (anchor_caps_)(caps | anchor_caps_stability);

	return caps;
}

void xr_ext_spatial_anchors_step() {
	for (int32_t i = 0; i < anchor_bridges.count; i++) {
		xr_anchor_bridge_t& bridge = anchor_bridges[i];
		if (!bridge.anchor || bridge.pending_create || bridge.entity_id == 0)
			continue;

		for (int32_t j = 0; j < local.anchors.count; j++) {
			if (local.anchors[j].entity_id == bridge.entity_id) {
				const xr_tracked_anchor_t& tracked = local.anchors[j];

				if (memcmp(&bridge.anchor->pose, &tracked.pose, sizeof(pose_t)) != 0) {
					bridge.anchor->pose = tracked.pose;
					anchor_mark_dirty(bridge.anchor);
				}

				button_state_ new_state = tracking_state_to_button(tracked.tracking_state);
				if (bridge.anchor->tracked != new_state) {
					bridge.anchor->tracked = new_state;
					anchor_mark_dirty(bridge.anchor);
				}

				if (bridge.anchor->persisted != tracked.is_persistent) {
					bridge.anchor->persisted = tracked.is_persistent;
					anchor_mark_dirty(bridge.anchor);
				}

				break;
			}
		}
	}
}

///////////////////////////////////////////

void xr_ext_spatial_entity_register();
xr_system_ xr_ext_spatial_entity_initialize(void*);
void       xr_ext_spatial_entity_shutdown  (void*);
void       xr_ext_spatial_entity_step_begin(void*);
void       xr_ext_spatial_entity_event_poll(void* context, XrEventDataBuffer *event_data);

void xr_ext_spatial_entity_register() {
	xr_system_t sys = {};
	sys.request_exts[sys.request_ext_count++] = XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME;
	sys.request_exts[sys.request_ext_count++] = XR_EXT_SPATIAL_ANCHOR_EXTENSION_NAME;
	sys.request_exts[sys.request_ext_count++] = XR_EXT_SPATIAL_PERSISTENCE_EXTENSION_NAME;
	sys.request_exts[sys.request_ext_count++] = XR_EXT_SPATIAL_PLANE_TRACKING_EXTENSION_NAME;
	sys.request_exts[sys.request_ext_count++] = XR_EXT_SPATIAL_MARKER_TRACKING_EXTENSION_NAME;
	sys.evt_initialize = { xr_ext_spatial_entity_initialize };
	sys.evt_shutdown   = { xr_ext_spatial_entity_shutdown   };
	sys.evt_step_begin = { xr_ext_spatial_entity_step_begin };
	sys.evt_poll       = { (void(*)(void*, void*))xr_ext_spatial_entity_event_poll };
	ext_management_sys_register(sys);
}

xr_system_ xr_ext_spatial_entity_initialize(void*) {
	if (!backend_openxr_ext_enabled(XR_EXT_SPATIAL_ENTITY_EXTENSION_NAME))
		return xr_system_fail;

	OPENXR_LOAD_FN_RETURN(XR_EXT_FUNCTIONS, xr_system_fail);

	uint32_t capability_count = 0;
	XrResult result = xrEnumerateSpatialCapabilitiesEXT(xr_instance, xr_system_id, 0, &capability_count, nullptr);
	if (XR_FAILED(result)) log_warnf("xrEnumerateSpatialCapabilitiesEXT [%s]", openxr_string(result));
	XrSpatialCapabilityEXT* capabilities = sk_malloc_t(XrSpatialCapabilityEXT, capability_count);
	result = xrEnumerateSpatialCapabilitiesEXT(xr_instance, xr_system_id, capability_count, &capability_count, capabilities);
	if (XR_FAILED(result)) log_warnf("xrEnumerateSpatialCapabilitiesEXT [%s]", openxr_string(result));

	local.capabilities = world_tracking_caps_none;
	log_info("Spatial Entity Capabilities:");
	for (uint32_t i = 0; i < capability_count; i++) {
		switch (capabilities[i]) {
		case XR_SPATIAL_CAPABILITY_ANCHOR_EXT:
			log_info("\tANCHOR");
			local.capabilities = (world_tracking_caps_)(local.capabilities | world_tracking_caps_anchor);
			break;
		case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_APRIL_TAG_EXT:
			log_info("\tAPRIL TAG");
			local.capabilities = (world_tracking_caps_)(local.capabilities | world_tracking_caps_marker_april);
			break;
		case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_ARUCO_MARKER_EXT:
			log_info("\tARUCO MARKER");
			local.capabilities = (world_tracking_caps_)(local.capabilities | world_tracking_caps_marker_aruco);
			break;
		case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_MICRO_QR_CODE_EXT:
			log_info("\tMICRO QR CODE");
			local.capabilities = (world_tracking_caps_)(local.capabilities | world_tracking_caps_marker_micro_qr);
			break;
		case XR_SPATIAL_CAPABILITY_MARKER_TRACKING_QR_CODE_EXT:
			log_info("\tQR CODE");
			local.capabilities = (world_tracking_caps_)(local.capabilities | world_tracking_caps_marker_qr);
			break;
		case XR_SPATIAL_CAPABILITY_PLANE_TRACKING_EXT:
			log_info("\tPLANE TRACKING");
			local.capabilities = (world_tracking_caps_)(local.capabilities | world_tracking_caps_surface);
			break;
		default: log_infof("\t%d", capabilities[i]);
		}

		XrSpatialCapabilityComponentTypesEXT capability_components = { XR_TYPE_SPATIAL_CAPABILITY_COMPONENT_TYPES_EXT };
		xrEnumerateSpatialCapabilityComponentTypesEXT(xr_instance, xr_system_id, capabilities[i], &capability_components);
		capability_components.componentTypes = sk_malloc_t(XrSpatialComponentTypeEXT, capability_components.componentTypeCapacityInput);
		xrEnumerateSpatialCapabilityComponentTypesEXT(xr_instance, xr_system_id, capabilities[i], &capability_components);

		for (uint32_t c = 0; c < capability_components.componentTypeCountOutput; c++) {
			switch (capability_components.componentTypes[c]) {
			case XR_SPATIAL_COMPONENT_TYPE_BOUNDED_2D_EXT:           log_info("\t\tcomponent - BOUNDED 2D");           break;
			case XR_SPATIAL_COMPONENT_TYPE_BOUNDED_3D_EXT:           log_info("\t\tcomponent - BOUNDED 3D");           break;
			case XR_SPATIAL_COMPONENT_TYPE_PARENT_EXT:               log_info("\t\tcomponent - PARENT");               break;
			case XR_SPATIAL_COMPONENT_TYPE_MESH_3D_EXT:              log_info("\t\tcomponent - MESH 3D");              break;
			case XR_SPATIAL_COMPONENT_TYPE_ANCHOR_EXT:               log_info("\t\tcomponent - ANCHOR");               break;
			case XR_SPATIAL_COMPONENT_TYPE_PERSISTENCE_EXT:          log_info("\t\tcomponent - PERSISTENCE");          break;
			case XR_SPATIAL_COMPONENT_TYPE_PLANE_ALIGNMENT_EXT:      log_info("\t\tcomponent - PLANE ALIGNMENT");      break;
			case XR_SPATIAL_COMPONENT_TYPE_MESH_2D_EXT:              log_info("\t\tcomponent - MESH 2D");              break;
			case XR_SPATIAL_COMPONENT_TYPE_POLYGON_2D_EXT:           log_info("\t\tcomponent - POLYGON 2D");           break;
			case XR_SPATIAL_COMPONENT_TYPE_PLANE_SEMANTIC_LABEL_EXT: log_info("\t\tcomponent - PLANE SEMANTIC LABEL"); break;
			case XR_SPATIAL_COMPONENT_TYPE_MARKER_EXT:               log_info("\t\tcomponent - MARKER");               break;
			default: log_infof("\t\tcomponent - %d", capability_components.componentTypes[c]); break;
			}
		}
		sk_free(capability_components.componentTypes);

		uint32_t feature_count = 0;
		result = xrEnumerateSpatialCapabilityFeaturesEXT(xr_instance, xr_system_id, capabilities[i], 0, &feature_count, nullptr);
		if (XR_SUCCEEDED(result) && feature_count > 0) {
			XrSpatialCapabilityFeatureEXT* capability_features = sk_malloc_t(XrSpatialCapabilityFeatureEXT, feature_count);
			xrEnumerateSpatialCapabilityFeaturesEXT(xr_instance, xr_system_id, capabilities[i], feature_count, &feature_count, capability_features);

			for (uint32_t f = 0; f < feature_count; f++) {
				switch (capability_features[f]) {
				case XR_SPATIAL_CAPABILITY_FEATURE_MARKER_TRACKING_FIXED_SIZE_MARKERS_EXT: log_info("\t\tfeature - FIXED SIZE MARKERS"); break;
				case XR_SPATIAL_CAPABILITY_FEATURE_MARKER_TRACKING_STATIC_MARKERS_EXT:     log_info("\t\tfeature - STATIC MARKERS");     break;
				default: log_infof("\t\tfeature - %d", capability_features[f]); break;
				}
			}
			sk_free(capability_features);
		}
	}
	sk_free(capabilities);

	// No base context creation - contexts created on-demand per capability category
	local.available = true;
	return xr_system_succeed;
}

void xr_ext_spatial_entity_shutdown(void*) {
	world_marker_stop();
	world_surface_stop();

	for (int32_t i = 0; i < spatial_category_max; i++) {
		if (local.contexts[i].context != XR_NULL_HANDLE) {
			xrDestroySpatialContextEXT(local.contexts[i].context);
			local.contexts[i].context = XR_NULL_HANDLE;
		}
	}

	for (int32_t i = 0; i < local.markers.count; i++) {
		if (local.markers[i].entity_handle != XR_NULL_HANDLE)
			xrDestroySpatialEntityEXT(local.markers[i].entity_handle);
		sk_free(local.markers[i].data);
	}
	local.markers.free();

	for (int32_t i = 0; i < local.planes.count; i++) {
		if (local.planes[i].entity_handle != XR_NULL_HANDLE)
			xrDestroySpatialEntityEXT(local.planes[i].entity_handle);
		local.planes[i].polygon.free();
		local.planes[i].mesh_vertices.free();
		local.planes[i].mesh_indices.free();
	}
	local.planes.free();

	for (int32_t i = 0; i < local.anchors.count; i++) {
		if (local.anchors[i].entity_handle != XR_NULL_HANDLE)
			xrDestroySpatialEntityEXT(local.anchors[i].entity_handle);
		local.anchors[i].mesh_vertices.free();
		local.anchors[i].mesh_indices.free();
	}
	local.anchors.free();

	if (local.persistence_context != XR_NULL_HANDLE) {
		xrDestroySpatialPersistenceContextEXT(local.persistence_context);
	}

	local = {};
	OPENXR_CLEAR_FN(XR_EXT_FUNCTIONS);
}

void xr_ext_spatial_entity_step_begin(void*) {
	// Nothing needed each frame - discovery triggered by events
}

void xr_ext_spatial_entity_event_poll(void*, XrEventDataBuffer* event_data) {
	if (event_data->type != XR_TYPE_EVENT_DATA_SPATIAL_DISCOVERY_RECOMMENDED_EXT)
		return;

	XrEventDataSpatialDiscoveryRecommendedEXT* recommendation = (XrEventDataSpatialDiscoveryRecommendedEXT*)event_data;

	for (int32_t i = 0; i < spatial_category_max; i++) {
		if (local.contexts[i].context == recommendation->spatialContext) {
			xr_spatial_context_begin_discovery((spatial_category_)i);
			return;
		}
	}
}

} // namespace sk
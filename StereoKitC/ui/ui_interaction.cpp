#include "ui_interaction.h"
#include "../sk_math.h"

#include <float.h>

namespace sk {

array_t<interactor_t> skui_interactors = { };

///////////////////////////////////////////

inline bounds_t size_box(vec3 top_left, vec3 dimensions) {
	return { top_left - dimensions / 2, dimensions };
}

///////////////////////////////////////////

void interactor_update_local(matrix to_local) {
	for (int32_t i = 0; i < skui_interactors.count; i++) {
		interactor_t* actor = &skui_interactors[i];
		
		actor->_hit_test_local      = matrix_transform_pose(to_local, actor->hit_test_world);
		actor->_hit_test_local_prev = matrix_transform_pose(to_local, actor->_hit_test_world_prev);
		actor->_motion_pose_local   = matrix_transform_pose(to_local, actor->motion_pose_world);
		if (actor->type == interactor_type_ray) {
			actor->_hit_test_local_dir = actor->_hit_test_local.orientation * vec3_forward;
		}
	}
}

///////////////////////////////////////////

bool32_t interactor_check_box(const interactor_t *actor, bounds_t box, vec3 *out_at, float *out_priority) {
	*out_priority = FLT_MAX;
	*out_at       = vec3_zero;

	if (!(actor->tracked & button_state_active))
		return false;

	if (skui_show_volumes)
		render_add_mesh(skui_box_dbg, skui_mat_dbg, matrix_trs(box.center, quat_identity, box.dimensions));

	switch (actor->type) {
	case interactor_type_point: {
		bool32_t result = bounds_capsule_contains(box, actor->_hit_test_local.position, actor->_hit_test_local_prev.position, actor->radius);
		if (result) {
			*out_at       = actor->_hit_test_local.position;
			*out_priority = bounds_sdf_manhattan(box, *out_at);
		}
		return result;
	} break;
	case interactor_type_ray: {
		ray_t    ray    = { actor->_hit_test_local.position, actor->_hit_test_local_dir };
		float    dist   = 0;
		bool32_t result = bounds_ray_intersect_dist(box, ray, &dist);
		if (result) {
			*out_at       = ray.pos + dist * ray.dir;
			*out_priority = bounds_sdf_manhattan(box, *out_at) + vec3_distance_sq(ray.pos, *out_at);
		}
		return result;
	} break;
	default: return false;
	}
}

///////////////////////////////////////////

void interactor_volume_1h(id_hash_t id, interactor_event_ event_mask, vec3 box_unfocused_start, vec3 box_unfocused_size, vec3 box_focused_start, vec3 box_focused_size, button_state_ *out_focus_state, int32_t *out_interactor, vec3 *out_interaction_at_local) {
	*out_interactor           = -1;
	*out_focus_state          = button_state_inactive;
	*out_interaction_at_local = vec3_zero;

	// If the element is disabled, leave it unfocused and ditch out
	if (!skui_interact_enabled) { return; }

	if (skui_preserve_keyboard_stack.last()) {
		skui_preserve_keyboard_ids_write->add(id);
	}

	for (int32_t i = 0; i < skui_interactors.count; i++) {
		const interactor_t *actor = &skui_interactors[i];
		if (((actor->events & event_mask) == 0) ||
			interactor_is_preoccupied(i, id, false))
			continue;

		bounds_t bounds = actor->focused_prev == id
			? size_box(box_focused_start,   box_focused_size)
			: size_box(box_unfocused_start, box_unfocused_size);

		float         priority = 0;
		vec3          interact_at;
		bool          in_box   = interactor_check_box(actor, bounds, &interact_at, &priority);
		button_state_ focus    = interactor_set_focus(i, id, in_box, priority);
		if (focus != button_state_inactive) {
			*out_interactor           = i;
			*out_focus_state          = focus;
			*out_interaction_at_local = interact_at;
		}
	}

	if (*out_interactor == -1)
		*out_interactor = interactor_last_focused(id);
}

///////////////////////////////////////////

void interactor_volume_2h(id_hash_t id, interactor_event_ event_mask, bounds_t bounds, ui_2h_state_* out_focus_state, int32_t* out_interactor1, int32_t* out_interactor2) {
	*out_focus_state = ui_2h_state_none;
	*out_interactor1 = -1;
	*out_interactor2 = -1;

	if (!skui_interact_enabled) {
		interactor_set_focus(-1, id, false, 0);
		return;
	}

	int32_t *out_interactor  [2] = { out_interactor1, out_interactor2 };
	float    interactor_focus[2] = { FLT_MAX, FLT_MAX };

	for (int32_t i = 0; i < skui_interactors.count; i++) {
		interactor_t *actor = &skui_interactors[i];
		// Skip this if something else has some focus!
		if (interactor_is_preoccupied(i, id, false) ||
			(actor->events & event_mask) == 0)
			continue;

		// Check to see if the handle has focus
		float hand_attention_dist = 0;
		vec3  interact_at;
		bool  has_hand_attention  = interactor_check_box(actor, bounds, &interact_at, &hand_attention_dist);
		button_state_ focused     = interactor_set_focus(i, id, has_hand_attention, hand_attention_dist);

		if (focused != button_state_inactive) {
			if (hand_attention_dist < interactor_focus[0]) {
				interactor_focus[1] = interactor_focus[0];
				*out_interactor [1] = *out_interactor[0];
				interactor_focus[0] = hand_attention_dist;
				*out_interactor [0] = i;
			}
		}

		// If this is the second frame this window has focus for, and it's at
		// a distance, then draw a line to it.
		/*if (hand_attention_dist && focused & button_state_active && !(focused & button_state_just_active)) {
			pointer_t *ptr   = input_get_pointer(input_hand_pointer_id[i]);
			vec3       start = hierarchy_to_local_point(ptr->ray.pos);
			line_add(start*0.75f, vec3_zero, { 50,50,50,0 }, { 255,255,255,255 }, 0.002f);
			from_pt = matrix_transform_pt(to_local, hierarchy_to_world_point(vec3_zero));
		}*/

		// This waits until the window has been focused for a frame,
		// otherwise the handle UI may try and use a frame of focus to move
		// around a bit.
		/*if (actor->focused_prev == id) {
			if (actor->state & button_state_just_active) {
				actor->_motion_pose_world_action = actor->motion_pose_world;
			}
			if (actor->active_prev == id || actor->active == id) {
				result = true;
				actor->active = id;
				actor->focused = id;

				quat dest_rot;
				vec3 dest_pos;

				// If both hands are interacting with this handle, then we do
				// a two handed interaction from the second hand.
				if (skui_hand[0].active_prev == id && skui_hand[1].active_prev == id || (skui_hand[0].active == id && skui_hand[1].active == id)) {
					if (i == 1) {
						dest_rot = quat_lookat(finger_pos[0], finger_pos[1]);
						dest_pos = finger_pos[0]*0.5f + finger_pos[1]*0.5f;

						if ((input_hand(handed_left)->pinch_state & button_state_just_active) || (input_hand(handed_right)->pinch_state & button_state_just_active)) {
							start_2h_pos = dest_pos;
							start_2h_rot = dest_rot;
							start_2h_handle_pos = movement.position;
							start_2h_handle_rot = movement.orientation;
						}

						switch (move_type) {
						case ui_move_exact: {
							dest_rot = quat_lookat(finger_pos[0], finger_pos[1]);
							dest_rot = quat_difference(start_2h_rot, dest_rot);
						} break;
						case ui_move_face_user: {
							dest_rot = quat_lookat(finger_pos[0], finger_pos[1]);
							dest_rot = quat_difference(start_2h_rot, dest_rot);
						} break;
						case ui_move_pos_only: {
							dest_rot = quat_identity;
						} break;
						case ui_move_none: {
							dest_rot = quat_identity;
						} break;
						default: dest_rot = quat_identity; log_err("Unimplemented move type!"); break;
						}

						hierarchy_set_enabled(false);
						line_add(matrix_transform_pt(to_world, finger_pos[0]), matrix_transform_pt(to_world, dest_pos), { 255,255,255,0 }, {255,255,255,128}, 0.001f);
						line_add(matrix_transform_pt(to_world, dest_pos), matrix_transform_pt(to_world, finger_pos[1]), { 255,255,255,128 }, {255,255,255,0}, 0.001f);
						hierarchy_set_enabled(true);

						dest_pos = dest_pos + dest_rot * (start_2h_handle_pos - start_2h_pos);
						dest_rot = start_2h_handle_rot * dest_rot;
						if (move_type == ui_move_none) {
							dest_pos = movement.position;
							dest_rot = movement.orientation;
						}

						movement.position    = vec3_lerp (movement.position,    dest_pos, 0.6f);
						movement.orientation = quat_slerp(movement.orientation, dest_rot, 0.4f);
					}

					// If one of the hands just let go, reset their starting
					// locations so the handle doesn't 'pop' when switching
					// back to 1-handed interaction.
					if ((input_hand(handed_left)->pinch_state & button_state_just_inactive) || (input_hand(handed_right)->pinch_state & button_state_just_inactive)) {
						start_handle_pos[i] = movement.position;
						start_handle_rot[i] = movement.orientation;
						start_palm_pos  [i] = from_pt;
						start_palm_rot  [i] = matrix_transform_quat( to_local, hand->palm.orientation);
					}
				} else {
					switch (move_type) {
					case ui_move_exact: {
						dest_rot = matrix_transform_quat(to_local, hand->palm.orientation);
						dest_rot = quat_difference(start_palm_rot[i], dest_rot);
					} break;
					case ui_move_face_user: {
						vec3 look_from = vec3{ movement.position.x, finger_pos[i].y, movement.position.z };
						dest_rot = quat_lookat_up(look_from, matrix_transform_pt(to_local, input_head()->position), matrix_transform_dir(to_local, vec3_up));
						dest_rot = quat_difference(start_handle_rot[i], dest_rot);
					} break;
					case ui_move_pos_only: {
						dest_rot = quat_identity;
					} break;
					case ui_move_none: {
						dest_rot = quat_identity;
					} break;
					default: dest_rot = quat_identity; log_err("Unimplemented move type!"); break;
					}

					vec3 curr_pos = finger_pos[i];
					dest_pos = curr_pos + dest_rot * (start_handle_pos[i] - start_palm_pos[i]);
					if (move_type == ui_move_none) dest_pos = movement.position;

					movement.position    = vec3_lerp (movement.position,    dest_pos, 0.6f);
					movement.orientation = quat_slerp(movement.orientation, start_handle_rot[i] * dest_rot, 0.4f); 
				}

				if (actor->state & button_state_just_inactive) {
					actor->active = 0;
				}
			}
		}*/
	}
}

///////////////////////////////////////////

bool32_t interactor_is_preoccupied(interactor_id_t interactor, id_hash_t for_el_id, bool32_t include_focused) {
	const interactor_t *actor = &skui_interactors[interactor];
	return (include_focused &&  actor->focused_prev != 0 && actor->focused_prev != for_el_id)
	                        || (actor->active_prev  != 0 && actor->active_prev  != for_el_id);
}

///////////////////////////////////////////

button_state_ interactor_set_focus(interactor_id_t interactor, id_hash_t for_el_id, bool32_t focused, float priority) {
	if (interactor == -1) return button_state_inactive;

	interactor_t *actor = &skui_interactors[interactor];
	bool was_focused = actor->focused_prev == for_el_id;
	bool is_focused  = false;

	/*if (hand == -1) {
		if      (skui_hand[0].active_prev == for_el_id) hand = 0;
		else if (skui_hand[1].active_prev == for_el_id) hand = 1;
	}*/
	if (focused && priority <= actor->focused_priority) {
		is_focused = focused;
		actor->focused          = for_el_id;
		actor->focused_priority = priority;
	}

	button_state_ result = button_state_inactive;
	if ( is_focused                ) result  = button_state_active;
	if ( is_focused && !was_focused) result |= button_state_just_active;
	if (!is_focused &&  was_focused) result |= button_state_just_inactive;
	return result;
}

///////////////////////////////////////////

button_state_ interactor_set_active(interactor_id_t interactor, id_hash_t for_el_id, bool32_t active, vec3 at) {
	if (interactor == -1) return button_state_inactive;

	interactor_t *actor = &skui_interactors[interactor];
	bool was_active = actor->active_prev == for_el_id;
	bool is_active  = false;

	if (active && (was_active || actor->focused_prev == for_el_id || actor->focused == for_el_id)) {
		is_active = active;
		actor->active = for_el_id;
	}

	button_state_ result = button_state_inactive;
	if ( is_active               ) result  = button_state_active;
	if ( is_active && !was_active) result |= button_state_just_active;
	if (!is_active &&  was_active) result |= button_state_just_inactive;

	if (result & button_state_just_active) {
		actor->_motion_pose_world_action = actor->motion_pose_world;
		actor->_hit_test_world_action    = actor->hit_test_world;
		actor->_hit_at_world             = hierarchy_to_world_point(at);
		actor->_hit_at_action_local      = matrix_transform_pt(matrix_invert(pose_matrix(actor->motion_pose_world)), actor->_hit_at_world);
	}
	
	return result;
}

///////////////////////////////////////////

interactor_id_t interactor_last_active(id_hash_t for_el_id) {
	return skui_interactors.index_where(&interactor_t::active_prev, for_el_id);
}

///////////////////////////////////////////

interactor_id_t interactor_last_focused(id_hash_t for_el_id){
	return skui_interactors.index_where(&interactor_t::focused_prev, for_el_id);
}

}
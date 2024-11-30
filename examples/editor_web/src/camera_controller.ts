import * as gl_matrix from "gl-matrix";
import { modify_camera, pick_entity } from "editor";
import * as log from "loglevel";

export class CameraController {
    window_size: gl_matrix.vec2;
    camera_pos: gl_matrix.vec2;
    camera_scale: number;
    current_scale: number;
    current_pos: gl_matrix.vec2;
    last_pos: gl_matrix.vec2;
    input_overlay: HTMLElement;
    enabled: boolean;
    left_button_down: boolean;
    constructor(overlay: HTMLElement) {
        const rect = overlay.getBoundingClientRect();
        this.window_size = gl_matrix.vec2.fromValues(rect.width, rect.height);
        this.current_pos = gl_matrix.vec2.fromValues(0, 0);
        this.last_pos = gl_matrix.vec2.fromValues(0, 0);
        this.camera_pos = gl_matrix.vec2.fromValues(0, 0);
        this.camera_scale = 1.0;
        this.current_scale = 1.0;
        this.input_overlay = overlay;
        this.enabled = true;
        this.left_button_down = false;
        this.register_inputs();
    }
    register_inputs() {
        this.input_overlay.addEventListener("dblclick", (event) => {
            if (event.button === 0) {
                log.warn("clicked");
                let pos = this.camera_to_world(gl_matrix.vec2.fromValues(event.clientX, event.clientY));
                pick_entity(pos[0], pos[1]).then((value: any) => { log.warn(value) });
            }
            event.preventDefault();
        });
        this.input_overlay.addEventListener("mousedown", (event) => {
            log.info("mouse down");
            if (event.button === 0 && this.enabled) {
                this.left_button_down = true;
                this.current_pos[0] = event.clientX;
                this.current_pos[1] = event.clientY;
            }
            event.preventDefault();
        });
        this.input_overlay.addEventListener("mousemove", (event) => {
            if (this.left_button_down && this.enabled) {
                gl_matrix.vec2.copy(this.last_pos, this.current_pos);
                this.current_pos[0] = event.clientX;
                this.current_pos[1] = event.clientY;
                this.modify_camera(true);
            }
            event.preventDefault();
        });
        this.input_overlay.addEventListener("mouseup", (event) => {
            log.info("mouse up");
            if (event.button === 0 && this.enabled) {
                this.left_button_down = false;
                gl_matrix.vec2.copy(this.last_pos, this.current_pos);
            }
            event.preventDefault();
        });
        this.input_overlay.addEventListener("wheel", (event) => {
            if (this.enabled) {
                this.current_scale += event.deltaY * 0.0079;
                this.modify_camera(true);
            }
            event.preventDefault();
        });
    }
    modify_camera(fore_modify: boolean) {
        let delta = gl_matrix.vec2.create();
        gl_matrix.vec2.subtract(delta, this.current_pos, this.last_pos);
        this.camera_scale = Math.pow(1.15, this.current_scale) / 1.15;
        this.camera_pos[0] -= this.camera_scale * delta[0];
        //we got y axies of different direction
        this.camera_pos[1] += this.camera_scale * delta[1];

        log.info("pos" + this.camera_pos, "scale" + this.camera_scale);
        modify_camera(this.camera_pos[0], this.camera_pos[1], this.camera_scale);
    }
    camera_to_world(pos: gl_matrix.vec2): gl_matrix.vec2 {
        let result = gl_matrix.vec2.create();
        gl_matrix.vec2.multiply(result, this.window_size, gl_matrix.vec2.fromValues(0.5, 0.5));
        gl_matrix.vec2.sub(result, pos, result);
        gl_matrix.vec2.mul(result, result, gl_matrix.vec2.fromValues(this.camera_scale, -1.0 * this.camera_scale));
        gl_matrix.vec2.add(result, result, this.camera_pos);
        return result;
    }
}

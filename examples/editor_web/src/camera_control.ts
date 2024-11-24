import * as gl_matrix from "gl-matrix";
import {modify_camera} from "editor";
import * as log from "loglevel";

export class CameraController{
    camera_pos: gl_matrix.vec2;
    camera_scale: number; 
    current_pos: gl_matrix.vec2;
    last_pos: gl_matrix.vec2;
    canvas: HTMLCanvasElement;
    enabled: boolean;
    left_button_down: boolean;
    constructor(canvas: HTMLCanvasElement){
        this.current_pos = gl_matrix.vec2.fromValues(0,0);
        this.last_pos = gl_matrix.vec2.fromValues(0,0);
        this.camera_pos = gl_matrix.vec2.fromValues(0,0);
        this.canvas = canvas;
        this.enabled = true;
        this.left_button_down = false;
        this.register_canvas_inputs();
    }
    register_canvas_inputs(){
        const overlay = document.getElementById("overlay");
        overlay.addEventListener('mousedown', function(event) {
            log.info('Mouse down at:', event.clientX, event.clientY);
        });
        
        overlay.addEventListener('mouseup', function(event) {
            log.info('Mouse up at:', event.clientX, event.clientY);
        });
        
        overlay.addEventListener('mousemove', function(event) {
            log.info('Mouse move at:', event.clientX, event.clientY);
        });
        // canvas.addEventListener("click", (event) => { });
        // document.addEventListener("mousedown", (event) => {
        //     log.info("mouse down");
        //     if (event.button === 0 && this.enabled){
        //         this.left_button_down = true;
        //     }
        // });
        // document.addEventListener("mousemove", (event) => {
        //     log.info("mouse moved");
        //     this.last_pos = this.current_pos;
        //     this.current_pos[0] = event.clientX;
        //     this.current_pos[1] = event.clientY;
        //     this.move_camera();
        // });
        // document.addEventListener("mouseup", (event) => {
        //     log.info("mouse up");
        //     if (event.button === 0 && this.enabled) {
        //         this.left_button_down = false;
        //     }
        // });
        // canvas.addEventListener("wheel", (event) => {
        // });
    }
    move_camera() {
        if(this.enabled && this.left_button_down){
            let delta = gl_matrix.vec2.create();
            gl_matrix.vec2.subtract(delta, this.camera_pos, this.last_pos);
            gl_matrix.vec2.add(this.camera_pos, this.camera_pos, delta);
            log.info("move camera" + this.camera_pos);
            modify_camera(this.camera_pos[0],this.camera_pos[1],this.camera_scale);
        }
    }

}

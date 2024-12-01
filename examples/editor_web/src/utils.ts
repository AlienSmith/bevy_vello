import * as dat from "dat.gui";
import * as gl_matrix from "gl-matrix";
export abstract class GUIWrapper {
  gui: dat.GUI | null = null;
  parent_gui: dat.GUI | null = null;
  destroy() {
    if (this.gui !== null && this.parent_gui !== null) {
      this.parent_gui.removeFolder(this.gui);
      this.gui = null;
    }
  }
}

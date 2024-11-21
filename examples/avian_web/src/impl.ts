import init from "avian";
import * as dat from "dat.gui";
init()
let obj = {
    load: () => {console.log("loaded")}
}

let gui = new dat.GUI();
gui.add(obj, "load");
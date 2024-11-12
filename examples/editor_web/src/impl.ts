import init, {load_assets_from_bytes} from "editor";
import * as dat from "dat.gui";
init()
let obj = {
    load: () => {console.log("loaded")}
}

let gui = new dat.GUI();
gui.add(obj, "load");

function loadExternalTraceFiles(files: FileList) {
    if (files.length === 1) {
        let name = files[0].name;
        let [_, ...remain] = name.split(".");
        if (remain.length != 1 || (remain[0] != "svg" && remain[0] != "json")){
            alert("invalid input, please provide a svg or lottie file");
            return;
        }
        files[0].arrayBuffer().then((bin) =>{
            let data = new Uint8Array(bin);
            alert("file provided" + name);
            load_assets_from_bytes(name, data).then((value: number) => {
                alert("asset mounted" + value);
            });
        } )
    }
}

function removeDragData(ev: DragEvent) {
    console.log("Removing drag data");
  
    if (ev.dataTransfer.items) {
      // Use DataTransferItemList interface to remove the drag data
      ev.dataTransfer.items.clear();
    }
}
  
function onDragOver(ev: DragEvent) {
    ev.preventDefault();
}
class DropWrapper {
    f: (file: FileList) => void;
    constructor(f: (file: FileList) => void) {
      this.f = f;
    }
    ondrop(ev: DragEvent) {
      console.log("File(s) dropped");
  
      // Prevent default behavior (Prevent file from being opened)
      ev.preventDefault();
      this.f(ev.dataTransfer.files);
      // Pass event to removeDragData for cleanup
      removeDragData(ev);
      return 0;
    }
  
}
let drop = new DropWrapper(loadExternalTraceFiles)
document.ondragover = onDragOver;
document.ondrop = (ev: DragEvent) => {
    drop.ondrop(ev);
};
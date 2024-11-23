import init, {start, load_lottie_assets_from_bytes, load_svg_assets_from_bytes, remove_entity, modify_entity, spawn_entity, Transform2D} from "editor";
import * as dat from "dat.gui";
import { GUIController } from "dat.gui";
Error.stackTraceLimit = 100;
init().then(()=>{start();});

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

class EntityItem extends GUIWrapper{
    name: string;
    id: number;
    x: number = 0;
    y: number = 0;
    r: number = 0;
    s_x: number = 1;
    s_y: number = 1;
    depth: number = 0;
    constructor(name: string, id:number, gui:dat.GUI){
        super()
        this.parent_gui = gui;
        this.name = name;
        this.id = id;
        this.x = 0;
        this.y = 0;
        this.gui = gui.addFolder("entity_" + name);
        let transform_gui = this.gui.addFolder("Transform");
        let temp = new Array<GUIController<object>>();
        temp.push(transform_gui.add(this,"x", -100, 100));
        temp.push(transform_gui.add(this,"y", -100, 100));
        temp.push(transform_gui.add(this,"s_x", 0.01, 10));
        temp.push(transform_gui.add(this,"s_y", 0.01, 10));
        temp.push(transform_gui.add(this,"r", 0, 10));
        temp.push(transform_gui.add(this,"depth", 0, 10));
        temp.forEach((item)=>{
            item.onChange(()=>{this.setEntityTransform()});
        });
        this.gui.add(this, "entityRemove");
        this.gui.open();
    }
    entityRemove(){
        remove_entity(this.id);
        this.destroy();
    }
    setEntityTransform(){
        console.log("x:" + this.x + "y:" + this.y)
        modify_entity(this.id, new Transform2D(this.x,this.y,this.r,this.s_x,this.s_y,this.depth));
    }
}

class AssetItem extends GUIWrapper{
    name: string;
    id: number;
    entities: Array<EntityItem>;
    constructor(name: string, id:number, gui:dat.GUI){
        super()
        this.parent_gui = gui;
        this.name = name;
        this.id = id;
        this.gui = gui.addFolder("asset_" + name);
        this.gui.add(this, "entitySpawn");
        this.gui.open();
        this.entities = new Array<EntityItem>();
    }
    entitySpawn(){
        spawn_entity(this.id, new Transform2D(100,0,0,1,1,0)).then((entity_id) =>{
            this.entities.push(new EntityItem(this.name + entity_id, entity_id,this.gui));
        });
    }
}

class Application {
    gui:dat.GUI;
    drop: DropWrapper;
    assets: Array<AssetItem>;
    constructor(){  
        this.gui = new dat.GUI();
        this.assets = new Array<AssetItem>();
        this.drop = new DropWrapper((files:FileList) =>{
            this.loadExternalTraceFiles(files);
        })
        document.ondragover = onDragOver;
        document.ondrop = (ev: DragEvent) => {
            this.drop.ondrop(ev);
        };
    }

    loadSVGAsset(name:string, data: Uint8Array) {
        load_svg_assets_from_bytes(data).then((id) =>{
            this.assets.push(new AssetItem(name, id, this.gui))
        });
    }

    loadLottieAsset(name:string, data: Uint8Array) {
        load_lottie_assets_from_bytes(data).then((id) =>{
            this.assets.push(new AssetItem(name, id, this.gui))
        });
    }

    loadExternalTraceFiles(files: FileList) {
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
                if (remain[0] == "svg"){
                    this.loadSVGAsset(name, data);
                }else if (remain[0] == "json") {
                    this.loadLottieAsset(name, data);
                }
            } )
        }
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

new Application()
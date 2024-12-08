import init, { start, load_lottie_assets_from_bytes, load_svg_assets_from_bytes, remove_entity, modify_entity, spawn_entity, Transform2D, load_particle_assets_from_bytes } from "editor";
import * as dat from "dat.gui";
import { GUIController } from "dat.gui";
import { GUIWrapper } from "./utils";
import log from 'loglevel';
import { CameraController } from "./camera_controller";

Error.stackTraceLimit = 100;

log.setLevel(log.levels.INFO);
let app: null | Application = null;
const overlay = document.getElementById("overlay");
init().then(() => { log.info("WASM LOADED"); app = new Application(overlay); start(); });

class EntityContainer extends GUIWrapper {
    entities: Map<number, EntityItem>;
    last_id: number;
    constructor(gui: dat.GUI) {
        super();
        this.parent_gui = gui;
        this.gui = gui.addFolder("Entities");
        this.entities = new Map<number, EntityItem>();
        this.last_id = 0;
    }
    addEntity(name: string, id: number) {
        this.entities.set(id, new EntityItem(name, id, this.gui, (id: number) => { this.removeEntity(id) }));
        this.presentGui(id);
    }
    removeEntity(id: number) {
        if (this.last_id === id) {
            this.last_id = 0;
        }
        this.entities.delete(id);
    }
    removeLastGui() {
        if (this.last_id != 0) {
            this.entities.get(this.last_id).remove_gui();
            this.last_id = 0;
        }
    }
    presentGui(entity_id: number) {
        if (this.last_id === entity_id) {
            return;
        }
        this.removeLastGui();
        this.entities.get(entity_id).build_gui();
        this.last_id = entity_id;
    }
}

class EntityItem extends GUIWrapper {
    name: string;
    id: number;
    x: number = 0;
    y: number = 0;
    r: number = 0;
    s_x: number = 1;
    s_y: number = 1;
    depth: number = 0;
    remove_this_from_collection: (id: number) => void;

    constructor(name: string, id: number, gui: dat.GUI, remove_this: (id: number) => void) {
        super()
        this.parent_gui = gui;
        this.name = name;
        this.id = id;
        this.x = 0;
        this.y = 0;
        this.remove_this_from_collection = remove_this;
    }

    build_gui() {
        this.gui = this.parent_gui.addFolder("entity_" + this.name);
        let transform_gui = this.gui.addFolder("Transform");
        let temp = new Array<GUIController<object>>();
        temp.push(transform_gui.add(this, "x", -100, 100));
        temp.push(transform_gui.add(this, "y", -100, 100));
        temp.push(transform_gui.add(this, "s_x", 0.01, 10));
        temp.push(transform_gui.add(this, "s_y", 0.01, 10));
        temp.push(transform_gui.add(this, "r", 0, 10));
        temp.push(transform_gui.add(this, "depth", 0, 10));
        temp.forEach((item) => {
            item.onChange(() => { this.setEntityTransform() });
        });
        this.gui.add(this, "entityRemove");
        this.gui.open();
        transform_gui.open();
    }

    remove_gui() {
        this.destroy_gui()
    }

    entityRemove() {
        this.destroy_gui();
        this.remove_this_from_collection(this.id);
        remove_entity(this.id);
    }

    setEntityTransform() {
        log.info("x:" + this.x + "y:" + this.y)
        modify_entity(this.id, new Transform2D(this.x, this.y, this.r, this.s_x, this.s_y, this.depth));
    }
}

class AssetItem extends GUIWrapper {
    name: string;
    id: number;
    is_particle: boolean;
    secondary_id: number;
    addEntityToContainer: (name: string, id: number) => void;
    constructor(name: string, id: number, gui: dat.GUI, is_particle: boolean, add_entity_to_container: (name: string, id: number) => void) {
        super()
        this.parent_gui = gui;
        this.name = name;
        this.id = id;
        this.is_particle = is_particle;
        this.secondary_id = 0;
        this.gui = gui.addFolder("asset_" + name + "__:" + id);
        this.gui.add(this, "secondary_id");
        this.gui.add(this, "entitySpawn");
        this.gui.open();
        this.addEntityToContainer = add_entity_to_container;
        log.info(`Asset created ............`)
    }
    entitySpawn() {
        spawn_entity(this.id, new Transform2D(0, 0, 0, 1, 1, 0), this.is_particle, this.secondary_id).then((entity_id) => {
            this.addEntityToContainer(this.name + entity_id, entity_id);
        });
    }
}

class Application {
    gui: dat.GUI;
    drop: DropWrapper;
    assets: Array<AssetItem>;
    entities: EntityContainer;
    input_overlay: HTMLElement;
    camera_controller: CameraController;
    constructor(overlay: HTMLElement) {
        this.gui = new dat.GUI();
        this.assets = new Array<AssetItem>();
        this.input_overlay = overlay;
        this.drop = new DropWrapper((files: FileList) => {
            this.loadExternalTraceFiles(files);
        })
        document.ondragover = onDragOver;
        document.ondrop = (ev: DragEvent) => {
            this.drop.ondrop(ev);
        };
        this.camera_controller = new CameraController(this.input_overlay, (id: number) => { this.entities.presentGui(id) });
        this.entities = new EntityContainer(this.gui);
        log.info("Application Created")
    }

    loadSVGAsset(name: string, data: Uint8Array) {
        load_svg_assets_from_bytes(data).then((id) => {
            this.assets.push(new AssetItem(name, id, this.gui, false, (name: string, id: number) => {
                this.addEntityToContainer(name, id);
            }));
        });
    }

    loadLottieAsset(name: string, data: Uint8Array) {
        load_lottie_assets_from_bytes(data).then((id) => {
            this.assets.push(new AssetItem(name, id, this.gui, false, (name: string, id: number) => {
                this.addEntityToContainer(name, id);
            }))
        });
    }

    loadParticleAsset(name: string, data: Uint8Array) {
        load_particle_assets_from_bytes(data).then((id) => {
            this.assets.push(new AssetItem(name, id, this.gui, true, (name: string, id: number) => {
                this.addEntityToContainer(name, id);
            }))
        });
    }

    addEntityToContainer(name: string, id: number) {
        this.entities.addEntity(name, id);
    }

    loadExternalTraceFiles(files: FileList) {
        if (files.length === 1) {
            let name = files[0].name;
            let [_, ...remain] = name.split(".");
            if (remain.length != 1 || (remain[0] != "svg" && remain[0] != "json" && remain[0] != "particles")) {
                alert("invalid input, please provide a svg, lottie or particles file");
                return;
            }
            files[0].arrayBuffer().then((bin) => {
                let data = new Uint8Array(bin);
                alert("file provided" + name);
                if (remain[0] == "svg") {
                    this.loadSVGAsset(name, data);
                } else if (remain[0] == "json") {
                    this.loadLottieAsset(name, data);
                } else if (remain[0] == "particles") {
                    this.loadParticleAsset(name, data);
                }
            })
        }
    }

}


function removeDragData(ev: DragEvent) {
    log.info("Removing drag data");

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
        log.info("File(s) dropped");

        // Prevent default behavior (Prevent file from being opened)
        ev.preventDefault();
        this.f(ev.dataTransfer.files);
        // Pass event to removeDragData for cleanup
        removeDragData(ev);
        return 0;
    }

}


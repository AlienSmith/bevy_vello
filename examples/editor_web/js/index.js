import React from 'react';
import ReactDOM from 'react-dom'

ReactDOM.render(<h1 class="title">Enjoy wgpu(WebGPU) + bevy + WASM + Webpack + React with 🍰☕</h1>, document.getElementById("root"));

import init from "editor";
window.addEventListener("load", () => {
  init();
});

#TO Setup WASM with webpack
1. Go to particles folder run "cargo build --release --target wasm32-unknown-unknown" 
2. Go back to root folder run "wasm-bindgen --out-dir out --target web target/wasm32-unknown-unknown/release/particles.wasm"
3. copy the out folder to webpack project
4. adjust the index.js if needed
5. Run "yarn run start" to serve from webpack dev server
6. Have fun 
fetch(' .. /out/main.wasm') . then(response =>
response.arrayBuffer()
). then(bytes => WebAssembly.instantiate(bytes)).then(results => {
instance = results.instance;
document.getElementById("container").innerText = instance.exports.add(1,1);
}).catch(console.error);



const fetchPromise = fetch('hello.wasm');
const importObj = {};

WebAssembly.instantiateStreaming(fetchPromise,
import0bj)
. then(result => {
// Use the WebAssembly Instance
const instance = result.instance;
});
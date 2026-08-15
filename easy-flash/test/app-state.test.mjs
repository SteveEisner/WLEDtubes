import assert from "node:assert/strict";
import test from "node:test";
import { initEasyFlash } from "../app.mjs";

class Element {
	constructor(doc){this.doc=doc;this.hidden=false;this.disabled=false;this.checked=false;this.textContent="";this.className="";this.listeners={};this.dataset={};}
	addEventListener(type,handler){this.listeners[type]=handler;}
	dispatch(type){return this.listeners[type]?.({target:this});}
	focus(){this.doc.activeElement=this;}
	append(){} replaceChildren(){}
}
function fixture({exactBoard=false}={}) {
	const doc={activeElement:null,documentElement:{dataset:{}},nodes:{},querySelector(selector){return this.nodes[selector];},createElement(){return new Element(this);}};
	for(const id of ["connect","install","physicalConfirmation","confirmedDig2Go","controllerState","controllerStatus","installTitle","instruction","actionNote","technicalDetails","resultPanel","resultTitle","resultMessage"]) doc.nodes[`#${id}`]=new Element(doc);
	doc.nodes["#install"].hidden=true;doc.nodes["#physicalConfirmation"].hidden=true;doc.nodes["#actionNote"].hidden=true;doc.nodes["#resultPanel"].hidden=true;
	const variant={id:"previous-stable-control",label:"Dig2Go",target:{board:"QuinLED Dig2Go"},artifacts:[]},artifact={transport:"usb",kind:"complete-merged-image"};variant.artifacts=[artifact];
	let installs=0,invalidator=null,lastArgs=null;
	const flash={setInvalidationHandler(handler){invalidator=handler;},async connectToController(){return {token:Object.freeze({id:"token"}),port:{id:"port"},chipName:"ESP32",portInfo:{usbVendorId:1,usbProductId:2},exactBoard};},async installConnectedController(args){installs++;lastArgs=args;return {chipName:"ESP32",sha256:"a".repeat(64),backup:"unavailable",readback:"writer-verified",health:"unverified"};}};
	const fetchImpl=async()=>({ok:true,json:async()=>({variants:[variant]})});const app=initEasyFlash({document:doc,fetchImpl,flash});
	return {doc,app,flash,counts:()=>installs,args:()=>lastArgs,disconnect:()=>invalidator?.()};
}

test("UI seam blocks unchecked confirmation and passes exact target-bound assertion when checked",async()=>{
	const f=fixture();await f.app.connect();assert.equal(f.doc.activeElement,f.doc.nodes["#confirmedDig2Go"]);assert.match(f.doc.nodes["#controllerStatus"].textContent,/confirm.*then choose Install/i);
	await f.app.install();assert.equal(f.counts(),0);
	f.doc.nodes["#confirmedDig2Go"].checked=true;f.doc.nodes["#confirmedDig2Go"].dispatch("change");await f.app.install();assert.equal(f.counts(),1);
	assert.deepEqual(f.args().physicalConfirmation,{asserted:true,targetId:"previous-stable-control",printedModel:"QuinLED Dig2Go"});assert.equal(f.args().sessionToken.id,"token");assert.equal(f.args().port.id,"port");
});

test("exact evidence focuses Install; disconnect resets confirmation and prevents stale reuse",async()=>{
	const f=fixture({exactBoard:true});await f.app.connect();assert.equal(f.doc.activeElement,f.doc.nodes["#install"]);assert.match(f.doc.nodes["#controllerStatus"].textContent,/choose Install/i);
	f.doc.nodes["#confirmedDig2Go"].checked=true;f.disconnect();assert.equal(f.doc.nodes["#confirmedDig2Go"].checked,false);assert.equal(f.doc.nodes["#install"].hidden,true);assert.equal(f.doc.nodes["#install"].disabled,true);assert.equal(f.doc.activeElement,f.doc.nodes["#connect"]);assert.match(f.doc.nodes["#controllerStatus"].textContent,/disconnected.*reconnect/i);await f.app.install();assert.equal(f.counts(),0);
});

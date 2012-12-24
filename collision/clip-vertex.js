// requires 
var b2Vec2 = require('../common/math/vec2.js'); 
var b2ContactID = require('./contact-id.js'); 
var ClipVertex = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
ClipVertex.prototype.__constructor = function(){}
ClipVertex.prototype.__varz = function(){
this.v =  new b2Vec2();
this.id =  new b2ContactID();
}
// static methods
// static attributes
// methods
ClipVertex.prototype.Set = function (other) {
		this.v.SetV(other.v);
		this.id.Set(other.id);
	}
// attributes
ClipVertex.prototype.v =  new b2Vec2();
ClipVertex.prototype.id =  new b2ContactID();// aliases  
ClipVertex.prototype.set = ClipVertex.prototype.Set; 
// exports  
module.exports =  ClipVertex;

// requires 
var b2Mat22 = require('../common/math/mat22.js'); 
var b2Vec2 = require('../common/math/vec2.js'); 
var b2OBB = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2OBB.prototype.__constructor = function(){}
b2OBB.prototype.__varz = function(){
this.R =  new b2Mat22();
this.center =  new b2Vec2();
this.extents =  new b2Vec2();
}
// static methods
// static attributes
// methods
// attributes
b2OBB.prototype.R =  new b2Mat22();
b2OBB.prototype.center =  new b2Vec2();
b2OBB.prototype.extents =  new b2Vec2();// aliases  
// exports  
module.exports =  b2OBB;

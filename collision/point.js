// requires 
var b2Vec2 = require('../common/math/vec2.js'); 
var b2Point = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Point.prototype.__constructor = function(){}
b2Point.prototype.__varz = function(){
this.p =  new b2Vec2();
}
// static methods
// static attributes
// methods
b2Point.prototype.Support = function (xf, vX, vY) {
		return this.p;
	}
b2Point.prototype.GetFirstVertex = function (xf) {
		return this.p;
	}
// attributes
b2Point.prototype.p =  new b2Vec2();// aliases  
b2Point.prototype.support = b2Point.prototype.Support; 
b2Point.prototype.getFirstVertex = b2Point.prototype.GetFirstVertex; 
// exports  
module.exports =  b2Point;

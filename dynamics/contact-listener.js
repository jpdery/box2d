// requires 
var b2ContactListener = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactListener.prototype.__constructor = function(){}
b2ContactListener.prototype.__varz = function(){
}
// static methods
// static attributes
b2ContactListener.b2_defaultListener =  new b2ContactListener();
// methods
b2ContactListener.prototype.BeginContact = function (contact) { }
b2ContactListener.prototype.EndContact = function (contact) { }
b2ContactListener.prototype.PreSolve = function (contact, oldManifold) {}
b2ContactListener.prototype.PostSolve = function (contact, impulse) { }
// attributes// aliases  
b2ContactListener.prototype.beginContact = b2ContactListener.prototype.BeginContact; 
b2ContactListener.prototype.endContact = b2ContactListener.prototype.EndContact; 
b2ContactListener.prototype.preSolve = b2ContactListener.prototype.PreSolve; 
b2ContactListener.prototype.postSolve = b2ContactListener.prototype.PostSolve; 
// exports  
module.exports =  b2ContactListener;

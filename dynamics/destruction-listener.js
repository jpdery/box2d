// requires 
var b2DestructionListener = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2DestructionListener.prototype.__constructor = function(){}
b2DestructionListener.prototype.__varz = function(){
}
// static methods
// static attributes
// methods
b2DestructionListener.prototype.SayGoodbyeJoint = function (joint) {}
b2DestructionListener.prototype.SayGoodbyeFixture = function (fixture) {}
// attributes// aliases  
b2DestructionListener.prototype.sayGoodbyeJoint = b2DestructionListener.prototype.SayGoodbyeJoint; 
b2DestructionListener.prototype.sayGoodbyeFixture = b2DestructionListener.prototype.SayGoodbyeFixture; 
// exports  
module.exports =  b2DestructionListener;

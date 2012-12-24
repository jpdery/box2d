// requires 
var b2Settings = require('../common/settings.js'); 
var b2Pair = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Pair.prototype.__constructor = function(){}
b2Pair.prototype.__varz = function(){
}
// static methods
// static attributes
b2Pair.b2_nullProxy =  b2Settings.USHRT_MAX;
b2Pair.e_pairBuffered =  0x0001;
b2Pair.e_pairRemoved =  0x0002;
b2Pair.e_pairFinal =  0x0004;
// methods
b2Pair.prototype.SetBuffered = function () { this.status |= b2Pair.e_pairBuffered; }
b2Pair.prototype.ClearBuffered = function () { this.status &= ~b2Pair.e_pairBuffered; }
b2Pair.prototype.IsBuffered = function () { return (this.status & b2Pair.e_pairBuffered) == b2Pair.e_pairBuffered; }
b2Pair.prototype.SetRemoved = function () { this.status |= b2Pair.e_pairRemoved; }
b2Pair.prototype.ClearRemoved = function () { this.status &= ~b2Pair.e_pairRemoved; }
b2Pair.prototype.IsRemoved = function () { return (this.status & b2Pair.e_pairRemoved) == b2Pair.e_pairRemoved; }
b2Pair.prototype.SetFinal = function () { this.status |= b2Pair.e_pairFinal; }
b2Pair.prototype.IsFinal = function () { return (this.status & b2Pair.e_pairFinal) == b2Pair.e_pairFinal; }
// attributes
b2Pair.prototype.userData =  null;
b2Pair.prototype.proxy1 =  null;
b2Pair.prototype.proxy2 =  null;
b2Pair.prototype.next =  null;
b2Pair.prototype.status =  0;// aliases  
b2Pair.prototype.setBuffered = b2Pair.prototype.SetBuffered; 
b2Pair.prototype.clearBuffered = b2Pair.prototype.ClearBuffered; 
b2Pair.prototype.isBuffered = b2Pair.prototype.IsBuffered; 
b2Pair.prototype.setRemoved = b2Pair.prototype.SetRemoved; 
b2Pair.prototype.clearRemoved = b2Pair.prototype.ClearRemoved; 
b2Pair.prototype.isRemoved = b2Pair.prototype.IsRemoved; 
b2Pair.prototype.setFinal = b2Pair.prototype.SetFinal; 
b2Pair.prototype.isFinal = b2Pair.prototype.IsFinal; 
// exports  
module.exports =  b2Pair;

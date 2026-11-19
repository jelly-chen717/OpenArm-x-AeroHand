// Fixed-axis roll/pitch/yaw: R = Rz(rz) * Ry(ry) * Rx(rx), angles in degrees.
export function quaternionToRPY(values){
 const n=Math.hypot(...values);if(!Number.isFinite(n)||n<1e-12)throw Error('Invalid quaternion');
 const [w,x,y,z]=values.map(v=>v/n);let pitch=Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))));
 let roll,yaw;
 // Stabilize float32 FK near gimbal lock (maximum angular change < 1e-5 rad).
 if(Math.abs(Math.cos(pitch))<1e-5){pitch=Math.sign(pitch)*Math.PI/2;roll=0;yaw=Math.atan2(2*(w*z-x*y),1-2*(x*x+z*z));}
 else{roll=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
 return [roll,pitch,yaw].map(v=>v*180/Math.PI);
}
export function rpyToQuaternion(values){
 const [r,p,y]=values.map(v=>v*Math.PI/360),cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
 return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy];
}

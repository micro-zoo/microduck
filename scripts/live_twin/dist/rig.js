import * as THREE from './vendor/three/three.module.js';
export function createPoseGraph(definition) {
  const joints = new Map(), bodies = new Map(), geometryParents = new Map();
  function visit(def) {
    const base = new THREE.Group(); base.name = def.name;
    base.position.fromArray(def.position);
    const [w,x,y,z] = def.quaternion; base.quaternion.set(x,y,z,w).normalize();
    let content=base;
    if(def.joint) {
      const pivot=new THREE.Group(); pivot.position.fromArray(def.joint.position); base.add(pivot);
      const after=new THREE.Group(); after.position.fromArray(def.joint.position).multiplyScalar(-1);pivot.add(after);
      joints.set(def.joint.name,{pivot,axis:new THREE.Vector3(...def.joint.axis).normalize(),id:def.joint.id,range:def.joint.range});
      content=after;
    }
    bodies.set(def.name,base);geometryParents.set(def.name,content);
    for(const child of def.children) content.add(visit(child));
    return base;
  }
  const root=visit(definition.root);
  const restRootQuaternion=root.quaternion.clone();
  return {root,joints,bodies,geometryParents,setTrunkOrientation(quat) {
    root.quaternion.set(quat[1],quat[2],quat[3],quat[0]).normalize().multiply(restRootQuaternion);
    root.updateMatrixWorld(true);
  },setAngles(angles) {
    for(const [name,joint] of joints) {const angle=angles[name];if(Number.isFinite(angle)) joint.pivot.quaternion.setFromAxisAngle(joint.axis,angle);}
    root.updateMatrixWorld(true);
  }};
}

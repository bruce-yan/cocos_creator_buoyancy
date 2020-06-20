/**
 * 流体浮力
 * 注意
 * 1.流体和进入流体的对象都必需使用cc.PhysicsPolygonCollider组件
 * 2.浮力跟流体、进入流体对象的密度有关
 */
const {ccclass, property} = cc._decorator;

@ccclass
export default class Water extends cc.Component {
    
    m_bodyArray: Array<cc.RigidBody> = new Array<cc.RigidBody>();
    //流体的刚体
    m_fluidBody: cc.RigidBody;
    //流体密度
    m_density = 1;

    onLoad () {
        this.m_fluidBody = this.getComponent(cc.RigidBody);
        this.m_density = this.getComponent(cc.PhysicsPolygonCollider).density;
    }
    
    start () {

    }

    update (dt) {
        this.m_bodyArray.forEach(body => {
            this.applyBuoyancy(body);
        });
    }

    // 只在两个碰撞体开始接触时被调用一次
    onBeginContact(contact:cc.PhysicsContact, selfCollider:cc.PhysicsPolygonCollider, otherCollider:cc.PhysicsPolygonCollider) {
        cc.log("onBeginContact");
        if(otherCollider.node.name == "box"){
            let body = otherCollider.node.getComponent(cc.RigidBody);
            
            //body.linearVelocity = cc.v2(body.linearVelocity.x * 0.7, body.linearVelocity.y * 0.3);
            //body.angularVelocity = body.angularVelocity * 0.7;
            this.m_bodyArray.push(body);    
        }
    }

    // 只在两个碰撞体结束接触时被调用一次
    onEndContact(contact:cc.PhysicsContact, selfCollider:cc.PhysicsPolygonCollider, otherCollider:cc.PhysicsPolygonCollider) {
        cc.log("onEndContact");
        if(otherCollider.node.name == "box"){
            let body:cc.RigidBody = otherCollider.node.getComponent(cc.RigidBody);
            let index = this.m_bodyArray.indexOf(body);
            if(index > -1){
                this.m_bodyArray.splice(index, 1);
            }  
        }        
    }
 
    applyBuoyancy(body:cc.RigidBody){
        let intersectionPoints = this.findIntersectionOfPolygon(body);
        if(intersectionPoints.length == 0){
            return;
        }

        let [area, centroid] = this.computeAC(intersectionPoints);
        let displacedMass = this.m_density * area;
        let gravity = cc.v2(0, -3);
        let buoyancyForce = cc.v2(displacedMass * -gravity.x, displacedMass * -gravity.y);
        body.applyForce(buoyancyForce, centroid, false);

        let lvBox = body.getLinearVelocityFromWorldPoint(centroid, null);
        let lvfluid = this.m_fluidBody.getLinearVelocityFromWorldPoint(centroid, null);
        let velDir = lvBox.subtract(lvfluid);
        let vel = velDir.len();
        velDir.normalizeSelf();

        let dragMag = this.m_density * vel * vel * 0.2;
        let dragForce = cc.v2(dragMag * -velDir.x, dragMag * -velDir.y);
        body.applyForce(dragForce, centroid, false);
        
        //let torque = -body.angularVelocity * 0.1;
        //body.applyTorque(torque, true);
        body.angularVelocity *= 0.7;
    }
    
    findIntersectionOfPolygon(body:cc.RigidBody):cc.Vec2[]{
        let outputList = this.getWorldVertices(this.m_fluidBody);
        let clipPolygon = this.getWorldVertices(body);

        let cp1 = clipPolygon[clipPolygon.length-1];
        for (let j = 0; j < clipPolygon.length; j++) {
            let cp2 = clipPolygon[j];
            if(outputList.length == 0){
                return [];
            }
            let inputList = outputList;
            outputList = [];
            let s = inputList[inputList.length - 1]; //last on the input list
            for (let i = 0; i < inputList.length; i++) {
                let e = inputList[i];
                if (this.inside(cp1, cp2, e)) {
                    if (!this.inside(cp1, cp2, s)) {
                        outputList.push(this.intersection(cp1, cp2, s, e));
                    }
                    outputList.push(e);
                }
                else if (this.inside(cp1, cp2, s)) {
                    outputList.push(this.intersection(cp1, cp2, s, e));
                }
                s = e;
            }
            cp1 = cp2;
        }        
        return outputList;
    }
    
    inside(cp1:cc.Vec2, cp2:cc.Vec2, p:cc.Vec2){
        return (cp2.x-cp1.x)*(p.y-cp1.y) > (cp2.y-cp1.y)*(p.x-cp1.x);
    }
    
    intersection(cp1:cc.Vec2, cp2:cc.Vec2, s:cc.Vec2, e:cc.Vec2){
        let dc = cc.v2(cp1.x-cp2.x, cp1.y-cp2.y);
        let dp = cc.v2(s.x-e.x, s.y-e.y);
        let n1 = cp1.x*cp2.y - cp1.y*cp2.x;
        let n2 = s.x * e.y - s.y * e.x;
        let n3 = 1.0/(dc.x * dp.y - dc.y * dp.x);
        return cc.v2((n1*dp.x-n2*dc.x)*n3, (n1*dp.y-n2*dc.y)*n3);
    }
    
    computeAC(vs:cc.Vec2[]):[number, cc.Vec2]{
        let count = vs.length;
        let c = cc.v2(0,0);
        let area = 0.0;
        let pRef = cc.v2(0.0, 0.0);
        let inv3 = 1.0 / 3.0;

        for (let i = 0; i < count; ++i) {
            let p1 = pRef;
            let p2 = vs[i];
            let p3 = i + 1 < count ? vs[i + 1] : vs[0];
            
            let e1 = p2.subtract(p1);
            let e2 = p3.subtract(p1);
            let D = e1.cross(e2);

            let triangleArea = 0.5 * D;
            area += triangleArea;
            c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x);
            c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y);
            //c += triangleArea * inv3 * (p1 + p2 + p3);
        }
        
        if (area > cc.macro.FLT_EPSILON){
            c.x *= 1.0 / area;
            c.y *= 1.0 / area;
        }
        else{
            area = 0;
        }

        return [area, c];
    }
    
    getWorldVertices(body: cc.RigidBody): cc.Vec2[]{
        let ppc = body.getComponent(cc.PhysicsPolygonCollider);
        if(!ppc){
            return [];
        }
        let ps = ppc.points;
        let vertices = [];
        for(let i = 0; i < ps.length; i++){
            vertices.push(body.getWorldPoint(ps[i], null));
        }
        return vertices;
    }    
}

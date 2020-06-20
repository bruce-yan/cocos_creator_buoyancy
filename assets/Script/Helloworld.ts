const {ccclass, property} = cc._decorator;

@ccclass
export default class Helloworld extends cc.Component {

    onLoad(){
        cc.director.getPhysicsManager().enabled = true;
        cc.director.getPhysicsManager().debugDrawFlags = cc.PhysicsManager.DrawBits.e_aabbBit |
            cc.PhysicsManager.DrawBits.e_jointBit |
            cc.PhysicsManager.DrawBits.e_shapeBit;
        //cc.director.getPhysicsManager().gravity = cc.v2(0, -20);
    }

    start () {
        // init logic
    }
}

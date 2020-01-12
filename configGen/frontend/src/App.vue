<template>
  <el-conatiner>
    <el-header>
      <h2>Nautilus config generator</h2>
    </el-header>
    <el-main>
      <div v-for="cate in table" :key="cate.alias">
        <el-card class="box-card" shadow="hover">
          <div slot="header" class="clearfix">
            <span>{{cate.alias}}</span>
            <el-button style="float: right; padding: 3px 0" type="text" @click="onSubmit">保存</el-button>
          </div>
          <el-form :ref="form[cate.alias]" :model="form[cate.alias]" label-width="200px">
            <el-form-item v-for="item in cate.child" :key="item.alias" :label="item.alias">
              <el-input v-model="form[cate.alias][item.alias]"></el-input>
            </el-form-item>
          </el-form>
        </el-card>
        <el-divider></el-divider>
      </div>
    </el-main>
  </el-conatiner>
</template>

<script>
let testJSON =
  '{\
  "serialPort": {\
    "portName": "COM3",\
    "baudrate": 230400,\
    "parity": "NoParity",\
    "dataBit": 8,\
    "stopBit": "OneStopBit",\
    "synchronize": true,\
    "syncInterval": 1\
  },\
  "camera": {\
    "deviceIndex": 0,\
    "frameWidth": 1280,\
    "frameHeight": 720\
  },\
  "testVideoSource": {\
    "filename": "test.avi"\
  },\
  "aimbot": {\
    "maxShootRadius": 120.0,\
    "enemyColor": "Red",\
    "brightness": 30,\
    "thresColor": 50,\
    "lightBarMinArea": 15.0,\
    "lightBarContourMinSolidity": 0.5,\
    "lightBarEllipseMinAspectRatio": 1.8,\
    "armorMaxAngleDiff": 7.5,\
    "armorMaxHeightDiffRatio": 0.6,\
    "armorMaxYDiffRatio": 2.0,\
    "armorMinXDiffRatio": 0.5,\
    "armorBigArmorRatio": 4.2,\
    "armorSmallArmorRatio": 2.0,\
    "armorMinAspectRatio": 1.0,\
    "armorMaxAspectRatio": 5.5,\
    "areaNormalizedBase": 1000.0,\
    "sightOffsetNormalizedBase": 200.0\
  },\
  "poseSolver": {\
    "offsetX": 0.0,\
    "offsetZPitch": 131.0,\
    "offsetYPitch": 50.0,\
    "offsetYYaw": -37.0,\
    "offsetZYaw": 213.0,\
    "offsetYaw": 0.0,\
    "offsetPitch": 0.55,\
    "initV": 20.0,\
    "initK": 0.026,\
    "gravity": 9.8\
  }\
}';

// import protobuf from "protobufjs";
import protoRoot from "./proto/config";
export default {
  name: "app",
  data() {
    return {
      form: {},
      table: []
    };
  },
  methods: {
    onSubmit() {
      console.log(JSON.stringify(this.form));
    },

    createTable(rootType) {
      let CheckEnum = typeName => {
        let isEnum = false;
        let values = [];
        try {
          let curt = protoRoot.lookupEnum(typeName);
          for (let item in curt.valuesById) {
            values.push(curt.valuesById[item]);
          }
          isEnum = true;
        } catch (err) {
          isEnum = false;
        }
        return { isEnum: isEnum, values: values };
      };

      let curtRoot = protoRoot.lookupType(rootType);
      let struct = curtRoot.toJSON().fields;
      let result = [];
      for (let item in struct) {
        try {
          result.push({
            alias: item,
            type: "object",
            child: this.createTable(struct[item].type)
          });
        } catch (err) {
          let { isEnum, values } = CheckEnum(struct[item].type);
          if (isEnum) {
            result.push({
              alias: item,
              type: "enum",
              values: values
            });
          } else {
            result.push({
              alias: item,
              type: "input"
            });
          }
        }
      }
      return result;
    },

    createTableContentObject(table) {
      let result = {};
      for (let item in table) {
        let alias = table[item].alias;
        let type = table[item].type;
        if (type === "object") {
          result[alias] = this.createTableContentObject(table[item].child);
        } else if (type === "enum") {
          result[alias] = "";
        } else if (type === "input") {
          result[alias] = "";
        }
      }
      return result;
    },

    Init() {
      // let config = protoRoot.lookupType("NautilusVisionConfig.Configuration");
      JSON.parse(testJSON);

      this.table = this.createTable("Configuration");
      console.log(this.table);

      this.form = this.createTableContentObject(this.table);
      console.log(this.form);
    }
  },
  created() {
    this.Init();
  }
};
</script>

<style>
</style>

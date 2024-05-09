<template>
  <div>
    <div class="buffer">
      <h1 class="text-center page-title">STORE INFO</h1>
    </div>
    <div class="container">
      <h1 class="text-center mt-5">매장안내</h1>
      <img src="@/assets/orange_line.png" alt="밑줄" class="orange-line">
      <div class="partner-container m-3">
        <h3 class="text-center m-1">{{ storename }} 빈자리 현황</h3>
        <div class="row">
          <div v-for="i in storelist" :key="i.id" class="col-lg-4 col-md-6 col-12">
            <div :class="{'box': true, 'active': i.coordinateX === '123'}">
              {{ i.roomNumber }} 번 룸 - {{ i.coordinateX }}
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import axios from 'axios';

export default {
  props: {
    storename: {
      type : String,
      required: true
    },
    storeid: {
      type : String,
      required: true
    }
  },
  data() {
    return {
      storelist: []
    };
  },
  mounted() {
    this.fetchRoomLists();
  },
  methods: {
    async fetchRoomLists() {
      try {
        const response = await axios.get(`https://k10a706.p.ssafy.io/api/stores/${this.storeid}/room-lists`);
        this.storelist = response.data; // 서버에서 받은 데이터로 storelist 업데이트
      } catch (error) {
        console.error("Room lists fetching error: ", error);
        // 에러 핸들링 코드 작성
      }
    }
  },
}

</script>

<style scoped>

.page-title {
  color: white;
  font-weight: 700;
  font-size: 50px;
  text-shadow: 2px 4px 4px rgba(0, 0, 0, 0.5);
}
.buffer {
  padding: 120px;
  background-image: url('@/assets/bgcard_dark.png');
  background-size: cover;
}
.container {
  padding: 20px;
  max-width: 1000px;
}

.orange-line{
  display: block;
  margin-left: auto;
  margin-right: auto;
  height: 6px;
  width: 60px;
}

.box {
  margin-top: 20px;
  margin-bottom: 20px;
  padding: 20px;
  border: 2px solid;
  border-radius: 10px;
  background-color: white;
}

.box.active {
  margin-top: 20px;
  margin-bottom: 20px;
  padding: 20px;
  border: 2px solid;
  border-radius: 10px;
  background-color: lightgrey;
}

.partner-container{
    border: 2px solid;
    border-radius: 10px;
    padding: 40px;
    background-color: antiquewhite;
}
</style>
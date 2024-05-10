<template>
  <div>
    <div class="buffer">
      <h1 class="text-center page-title">STORE INFO</h1>
    </div>
    <div class="container">
      <h1 class="text-center mt-5">매장안내</h1>
      <img src="@/assets/orange_line.png" alt="밑줄" class="orange-line">
      <div v-for="(branch, index) in branches" :key="index" class="map-container m-5">
        <div class="grid-container p-3">
          <KakaoMap :latitude="branch.latitude" :longitude="branch.longitude" :mapId="'map'+index" class="left-column"/>
          <div class="right-column p-4">
            <h3>{{ branch.storeName }}</h3>
            <p>오시는길: {{ branch.address }}</p>
            <p>영업시간: {{ branch.hours }}</p>
            <p>대표번호: {{ branch.phone }}</p>
            <router-link 
              :to="{ name: 'SeatsInfo', params: { storename: branch.storeName, storeid: branch.id }}"
              class="btn btn-primary">
              매장 현황 보기
            </router-link>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import axios from 'axios';
import KakaoMap from '@/components/KakaoMap.vue'

export default {
    components:{
        KakaoMap,
    },
    data() {
    return {
      branches: [
        // { id: "1", storename: "멀티캠퍼스 역삼점", latitude: 37.5012647456244, longitude: 127.03958123605, address: '역삼역 1번출구 역삼 멀티캠퍼스 15층', hours: '11:00 ~ 22:00', phone: '000-0000-0000' },
        // { id: "2", storename: "깊은 저 바닷속 파인애플점", latitude: 37.4992647456244, longitude: 129.13958123605, address: '뚱이네, 징징이네 옆집', hours: '11:00 ~ 22:00', phone: '000-0000-0000' },
        // { id: "3", storename: "강남역점", latitude: 37.4980647456244, longitude: 127.02875123605, address: '강남역 1번출구', hours: '오전 11:00 ~ 22:00', phone: '010-0000-0000' },
      ]
    };
  },
  mounted() {
    this.fetchStoreLists();
  },
  methods: {
    async fetchStoreLists() {
      try {
        const response = await axios.get(process.env.VUE_APP_API_URL + `stores`);
        this.branches = response.data; // 서버에서 받은 데이터로 storelist 업데이트
        console.log(this.branches)
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
.map-container {
  display: grid;
  border-color: black;
  border: 2px solid;
  border-radius: 5px;
}

.grid-container {
  display: grid;
  grid-template-columns: 1fr 2fr;
}

.table-in-use {
  background-color: lightcoral;
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
</style>
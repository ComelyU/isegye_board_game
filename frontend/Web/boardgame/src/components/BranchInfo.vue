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
            <h3>{{ branch.name }}</h3>
            <p>오시는길: {{ branch.address }}</p>
            <p>영업시간: {{ branch.hours }}</p>
            <p>대표번호: {{ branch.phone }}</p>
            <button type="button" class="btn btn-primary" @click="openModal">
              매장 현황 보기
            </button>
          </div>
        </div>
      </div>
      
      <div class="modal fade" id="storeModal" tabindex="-1" aria-labelledby="storeModalLabel" aria-hidden="true">
    
          <div class="modal-dialog modal-lg">
              <div class="modal-content">
                  <div class="modal-header">
                      <h5 class="modal-title" id="storeModalLabel">매장 현황</h5>
                      <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
                  </div>
                  <div class="modal-body">
                      <div v-for="table in tables" :key="table.id" :class="{ 'table-in-use': table.fcmToken }">
                        테이블 {{ table.id }}
                      </div>
                  </div>
                  <div class="modal-footer">
                      <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">닫기</button>
                  </div>
              </div>
          </div>
      </div>
    </div>
  </div>
</template>

<script>
import KakaoMap from '@/components/KakaoMap.vue'
import axios from 'axios';

export default {
    components:{
        KakaoMap
    },
    data() {
    return {
      branches: [
        { name: '멀티캠퍼스 역삼점', latitude: 37.5012647456244, longitude: 127.03958123605, address: '역삼역 1번출구 역삼 멀티캠퍼스 15층', hours: '11:00 ~ 22:00', phone: '000-0000-0000' },
        { name: '깊은 저 바닷속 파인애플점', latitude: 37.4992647456244, longitude: 129.13958123605, address: '뚱이네, 징징이네 옆집', hours: '5:00 ~ 13:00', phone: '000-0000-0000' },
        { name: '강남역점', latitude: 37.4980647456244, longitude: 127.02875123605, address: '강남역 1번출구', hours: '오전 11:00 ~ 22:00', phone: '010-0000-0000' },
      ]
    };
  },
    methods: {
        openModal() {
          console.log("응애")
            axios.get('https://k10a706.p.ssafy.io/api/stores/1/room-lists/available')
                .then(response => {
                  console.log(response)
                  // 받은 데이터를 테이블 목록으로 설정
                  this.tables = response.data.tables;
                })
                .catch(error => {
                  console.error('Error fetching tables:', error);
                });
        }
    }
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
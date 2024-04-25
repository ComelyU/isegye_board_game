<template>
  <div :id="mapId" style="height: 300px; width: 400px;"></div>
</template>

<script>
export default {
  name:"KakaoMap",
  props: {
    latitude: {
      type: Number,
      required: true,
    },
    longitude: {
      type: Number,
      required: true,
    },
    mapId: {
      type: String,
      required: true,
    },
  },
  data() {
    return {
      map: null
    }
  },
  mounted() {
    if (window.kakao && window.kakao.maps) {
      this.loadMap();
    }  else {
      this.loadScript();
    }
  },
  methods:{
    loadScript() {
      const script = document.createElement("script");
      const apiKey = process.env.VUE_APP_KAKAO_MAP_API_KEY;

      script.src=`//dapi.kakao.com/v2/maps/sdk.js?appkey=${apiKey}&autoload=false`
      script.onload = () => window.kakao.maps.load(this.loadMap); 

      document.head.appendChild(script);
    },
    loadMap() {
      const container = document.getElementById(this.mapId);
      const options = {
        //좌표값 설정
        center: new window.kakao.maps.LatLng(this.latitude, this.longitude),
        level: 4
      };

      this.map = new window.kakao.maps.Map(container, options);
      this.loadMaker();
    },
    loadMaker() {
      const markerPosition = new window.kakao.maps.LatLng(this.latitude, this.longitude);

      const marker = new window.kakao.maps.Marker({
        position:markerPosition
      });

      marker.setMap(this.map);
    }
  }
};
</script>

<style scoped>

</style>
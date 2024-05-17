<template>
  <div class="back-color">
    <div class="buffer"></div>
    <article class="whiteback content-box">
    <h3 class="bit-font">이세상 보드게임이 아니다!</h3>
    <p></p>
    <p>롸-벗이 직접 배달해주는 30년 전통 원조 보드게임카페</p>
    <p></p>
    <img src="@/assets/qr_cu.png" alt="qr코드 고객용" class="qrimage">
    <img src="@/assets/qr_ma.png" alt="qr코드 점주용" class="qrimage">
    </article>
  </div>
  
</template>

<script>
export default {
  name: 'MainPage',
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}

.back-color {
  /* background-image: url('assets/boardgameshop.png'); */
  background-image: url('@/assets/castle.png');
  background-size: cover;
  min-height: 150vh;
}

.whiteback {
  background-color: white;
}

.content-box {
  margin-top: 100px;
  margin-left: 200px;
  border-radius: 10px;
  padding: 20px;
  box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1);
  max-width: 500px;
}

.buffer {
  padding: 10px;
}

.bit-font {
  font-family: 'bitbit';
}

.qrimage {
  width: 450px;
  margin-top: 20px;
}
</style>
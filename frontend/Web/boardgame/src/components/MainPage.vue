<template>
  <div class="back-color">
    <div class="buffer"></div>
    
    <article class="content-box">
      <h1 class="bit-font">이세상 보드게임이 아니다!</h1>
      <p></p>
      <h3 class="dodum">배달로봇이 직접 배달해주는 30년 전통<br>원조 보드게임카페</h3>
      <div class="m-5 p-2"></div>
      <img src="@/assets/gamelist&order.gif" alt="game gif">
      <h3 class="dodum m-3 text-center">사용이 간편한 키오스크 앱 제공</h3>
      <div class="m-5 p-2"></div>
      <img src="@/assets/tutorial.gif" alt="game gif">
      <h3 class="dodum mt-3 text-center">처음 방문하는 고객을 위한 튜토리얼 제공</h3>
      <div class="m-5 p-4"></div>
      <div class="bottom">
        <h2 class="text-center dodum mt-3">이세계 보드게임 전용앱 다운링크</h2>
        <img src="@/assets/qr_cu.png" alt="qr코드 고객용" class="qrimage ms-5 me-5">
        <img src="@/assets/qr_ma.png" alt="qr코드 점주용" class="qrimage">
      </div>
    
    </article>
    
    <div class="parallelogram"></div>
    

    
  </div>
  
</template>

<script>
export default {
  name: 'MainPage',
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>

.dodum {
  font-family: 'dodum';
}

.bottom {
  align-self: flex-end;
}

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
  background-image: url('@/assets/castle.png');
  background-size: cover;
  min-height: 150vh;
  position: relative;
}

.content-box {
  margin-top: 100px;
  margin-left: 200px;
  /* border-radius: 10px; */
  /* padding: 20px; */
  /* box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1); */
  max-width: 510px;
  height: 90%;
  position: relative;
  z-index: 2;
  display: flex;
  flex-direction: column;
}

.buffer {
  padding: 10px;
}

.bit-font {
  font-family: 'bitbit';
}

.qrimage {
  width: 200px;

}

.parallelogram {
  position: absolute;
  top: 0;
  left: -30%;
  width: 70%;
  min-height: 150vh;
  background: rgba(255, 255, 255, 0.8); /* 흰색이면서 반투명한 배경색 */
  transform: skew(20deg); /* 평행사변형 만들기 위해 기울이기 */
  z-index: 1; 
}
</style>
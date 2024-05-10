<template>
    <div>
        <div class="buffer">
            <h1 class="text-center page-title">PARTNERSHIP</h1>
        </div>
        <div class="container">
            <h1 class="text-center mt-5">가맹 문의</h1>
            <img src="@/assets/orange_line.png" alt="밑줄" class="orange-line mb-5">
            <div class="partner-container">
                <h3 class="p-4 text-center partner-text">높아지는 최저임금, 부담스러운 가맹료 </h3>
                <h3 class="p-4 text-center partner-text">ISEGYE가 해결해 드리겠습니다!</h3>
                <h3 class="p-4 text-center partner-text">저희 ISEGYE는 점주여러분과 공생을 최대 가치로 생각합니다.</h3>
                <h3 class="p-4 text-center partner-text">지금바로 연락하세요!</h3>
                <div class="m-5 ps-5 pe-5">
                    <h4 class="text-center">CONTECT US</h4>
                    <p>대표 : 大 상 협</p>
                    <p>연락처 : 000-0000-0000</p>
                    <p>메일 : isegyeboardgame@isegye.com</p>
                    <p>본사 주소 : 서울시 강남구 테헤란로 212, 멀티캠퍼스 1501호</p>
                </div>
                
            </div>
        </div>
    </div>    
</template>

<script>
export default {
    name: 'PartnerShip',
}
</script>

<style scoped>

.buffer {
    padding: 120px;
    background-image: url('@/assets/bgcard_dark.png');
    background-size: cover;
}
.page-title {
    color: white;
    font-weight: 700;
    font-size: 50px;
    text-shadow: 2px 4px 4px rgba(0, 0, 0, 0.5);
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

.partner-container{
    border: 2px solid;
    border-radius: 10px;
    padding: 40px;
    background-color: antiquewhite;
}

.partner-text {
  font-family: 'yeonsung';
  font-size: 40px;
  color: black;
}
</style>
  
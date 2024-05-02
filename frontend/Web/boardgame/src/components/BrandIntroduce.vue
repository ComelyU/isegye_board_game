<template>
    <div>
        <div class="banner">
            <h1 class="text-center page-title">BRAND</h1>
        </div>
        <div class="container">
            <h1 class="text-center mt-5">브랜드 소개</h1>
            <img src="@/assets/orange_line.png" alt="밑줄" class="orange-line mb-5">
            <div id="brandintroducecarousel" class="carousel slide" data-bs-ride="carousel">
                <div class="carousel-indicators">
                    <button type="button" data-bs-target="#brandintroducecarousel" data-bs-slide-to="0" class="active" aria-current="true" aria-label="Slide 1"></button>
                    <button type="button" data-bs-target="#brandintroducecarousel" data-bs-slide-to="1" aria-label="Slide 2"></button>
                    <button type="button" data-bs-target="#brandintroducecarousel" data-bs-slide-to="2" aria-label="Slide 3"></button>
                </div>
                <div class="carousel-inner">
                    <div class="carousel-item active" data-bs-interval="10000">
                        <img src="@/assets/caro1.png" class="d-block w-100" alt="...">
                        <div class="carousel-caption d-none d-md-block">
                            <!-- <h1>원하는 보드게임을 자리까지</h1>
                            <p>Some representative placeholder content for the first slide.</p> -->
                        </div>
                    </div>
                <div class="carousel-item" data-bs-interval="10000">
                    <img src="@/assets/caro2.png" class="d-block w-100" alt="...">
                    <div class="carousel-caption d-none d-md-block">
                        <!-- <h1>결정장애 친구들을 위한 게임추천기능</h1>
                        <p>Some representative placeholder content for the second slide.</p> -->
                    </div>
                </div>
                <div class="carousel-item" data-bs-interval="10000">
                    <img src="@/assets/caro3.png" class="d-block w-100" alt="...">
                    <div class="carousel-caption d-none d-md-block">
                        <!-- <h1>테마와 함께 플레이 하고 기념사진까지</h1>
                        <p>Some representative placeholder content for the third slide.</p> -->
                    </div>
                </div>
            </div>
            <button class="carousel-control-prev" type="button" data-bs-target="#brandintroducecarousel" data-bs-slide="prev">
                <span class="carousel-control-prev-icon" aria-hidden="true"></span>
                <span class="visually-hidden">이전</span>
            </button>
                <button class="carousel-control-next" type="button" data-bs-target="#brandintroducecarousel" data-bs-slide="next">
                    <span class="carousel-control-next-icon" aria-hidden="true"></span>
                    <span class="visually-hidden">다음</span>
                </button>
            </div>
            <div class="buffer"></div>
            <h1 class="text-center mt-5">서비스 방식</h1>
            <img src="@/assets/orange_line.png" alt="밑줄" class="orange-line mb-5">
            <img src="@/assets/service_system.png" alt="통신이미지" class="intro-image">
            <ul class="m-5 p-2">
                <li>
                    <h2>음료부터 게임까지 모든 배송 자동화</h2>
                </li>
                <li>
                    <h2>알바생은 전부 짜르세요</h2>
                </li>
                <li>
                    <h2>자동으로 테마 제공</h2>
                </li>
            </ul>
            
        </div>
    </div>
</template>

<script>
export default {
    name: 'BrandIntroduce',
}
</script>

<style scoped>
.banner {
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

.orange-line{
    display: block;
    margin-left: auto;
    margin-right: auto;
    height: 6px;
    width: 60px;
}

.intro-image {
    max-width: 100%;
}

.buffer {
    padding: 30px;
}
.container {
    padding-top: 20px;
}
</style>
  
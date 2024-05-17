<template>
  <div >
    <header>
      <nav class="navbar navbar-expand-lg container">
        <div class="container-fluid">
          <div>
            <a class="navbar-brand" href="/">
              <img src="@/assets/Knight.png" height="70px">
              <span class="navbar-text brand-text align-middle">이세계보드게임</span>
            </a>            
          </div>

          <button class="navbar-toggler" type="button" data-bs-toggle="collapse"
          data-bs-target="#navbarSupportedContent" aria-controls="navbarSupportedContent"
          aria-expanded="false" aria-label="Toggle navigation">
            <span class="navbar-toggler-icon"></span>
          </button>   

          <div class="collapse navbar-collapse justify-content-end" id="navbarSupportedContent">
            <ul class="navbar-nav me-1 mb-2">
              <li class="nav-item me-4">
                <RouterLink :to="{ name: 'BrandIntroduce' }" class="nav-link-font">브랜드</RouterLink>
              </li>
              <li class="nav-item me-4">
                <RouterLink :to="{ name: 'BranchInfo' }" class="nav-link-font">지점안내</RouterLink>
              </li>           
              <li class="nav-item me-2">
                <RouterLink :to="{ name: 'PartnerShip' }" class="nav-link-font">가맹문의</RouterLink>
              </li>
            </ul>
          </div>
        </div>
      </nav>
    </header>
    <article>
      <RouterView />
    </article>
    <footer class="footer">
      <p>@ISEGYE ssafy A706 </p>
      <p>본사: 서울시 강남구 테헤란로 212, 멀티캠퍼스 1501호</p>
      <p>대표: 大 상 협</p>
    </footer>
  </div>  
</template>

<script>
export default {
  name: 'App'
}
</script>

<style>

.footer {
  bottom: 0;
  width: 100%;
  background-color: #f8f9fa;
  padding: 20px 0;
  text-align: center;
}

@font-face {
  font-family: 'bitbit';
  src: url('assets/fonts/DNFBitBitv2.ttf') format('truetype');
}

@font-face {
  font-family: 'yeonsung';
  src: url('assets/fonts/YeonSung.ttf') format('truetype');
}

.brand-text {
  font-family: 'bitbit';
  font-size: 40px;
  color: black;
}

.nav-link-font {
  font-family: 'jua';
  font-size: 30px;
  color: black;
  text-decoration-line: none;
}
</style>

# Node 21 Alpine 이미지를 기반으로 설정
FROM node:21-alpine

# 작업 디렉토리 설정
WORKDIR /app

# package.json 및 package-lock.json 파일을 /app 디렉토리로 복사
COPY package*.json ./

# 프로젝트 의존성 설치
RUN npm install

# .env 파일을 /app 디렉토리로 복사
COPY .env .

# 로컬 시스템의 현재 디렉토리에서 /app 디렉토리로 소스 코드 복사
COPY . .

# Vue.js 애플리케이션 빌드
RUN npm run build

# 3000 포트를 외부에 노출
EXPOSE 3000

# Vue.js 애플리케이션 실행
CMD ["npm", "run", "serve"]
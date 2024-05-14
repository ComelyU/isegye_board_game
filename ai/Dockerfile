FROM docker
COPY --from=docker/buildx-bin:latest /buildx /usr/libexec/docker/cli-plugins/docker-buildx

# 파이썬 3.10.6 버전을 기본 이미지로 사용합니다.
FROM python:3.10.6

# 작업 디렉토리 설정
WORKDIR /app

# 시스템 레벨 의존성 설치
RUN apt-get update && apt-get install -y \
    build-essential \
    libssl-dev \
    libffi-dev \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# pip를 최신 버전으로 업그레이드
RUN pip install --upgrade pip

# 의존성 파일을 먼저 복사
COPY requirements.txt .

# 의존성 설치
RUN pip install --no-cache-dir --upgrade -r requirements.txt

# 나머지 파일 복사
COPY . .

# 호스트와 연결할 포트 번호
EXPOSE 8000

# FastAPI 서버 실행
CMD ["fastapi", "run", "main.py", "--port", "8000"]

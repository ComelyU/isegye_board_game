FROM docker
COPY --from=docker/buildx-bin:latest /buildx /usr/libexec/docker/cli-plugins/docker-buildx

# 기본 이미지로 파이썬 3.10.6-slim 버전을 사용합니다.
FROM python:3.10.6-slim

# 작업 디렉토리 설정
WORKDIR /app

# 의존성 파일을 먼저 복사하여 캐시 효율성을 높입니다.
COPY requirements.txt .

# 의존성 설치
RUN pip install --no-cache-dir -r requirements.txt

# 나머지 파일 복사
COPY . .

# 호스트와 연결할 포트 번호
EXPOSE 8000

# FastAPI 서버 실행 (uvicorn 사용, 호스트는 외부 접속 허용을 위해 0.0.0.0으로 설정)
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]

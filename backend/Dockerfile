# docker buildx 이용
FROM docker
COPY --from=docker/buildx-bin:latest /buildx /usr/libexec/docker/cli-plugins/docker-buildx

# 기본 이미지로 eclipse temurin 기반 17 jdk 사용
FROM eclipse-temurin:17-jdk
# 애플리케이션 JAR 파일 복사
ADD ./build/libs/isegye-0.0.1-SNAPSHOT.jar app.jar
# 실행할 명령어 설정
ENTRYPOINT ["java", "-jar", "/app.jar"]
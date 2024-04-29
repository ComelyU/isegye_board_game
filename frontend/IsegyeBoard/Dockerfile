FROM docker
COPY --from=docker/buildx-bin:latest /buildx /usr/libexec/docker/cli-plugins/docker-buildx

FROM eclipse-temurin:17-jdk

ENV ANDROID_COMPILE_SDK="34" \
    ANDROID_SDK_BUILD_TOOLS="34.0.0" \
    ANDROID_SDK_PLATFORM_TOOLS="35.0.1" \
    ANDROID_SDK_COMMANDLINE_TOOLS="11076708"

RUN apt-get update && \
    apt-get install -y curl tar bash procps unzip && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /android-sdk

RUN curl -o cmdline-tools.zip "https://dl.google.com/android/repository/commandlinetools-linux-${ANDROID_SDK_COMMANDLINE_TOOLS}_latest.zip" && \
    unzip cmdline-tools.zip -d . && \ 
    rm cmdline-tools.zip && \
    mv cmdline-tools cmdline-tools-temp && \
    mkdir -p cmdline-tools/latest && \
    mv cmdline-tools-temp/* cmdline-tools/latest/ && \
    rm -r cmdline-tools-temp

ENV ANDROID_HOME=/android-sdk
ENV PATH $PATH:$ANDROID_HOME/cmdline-tools/latest/bin

RUN yes | sdkmanager --licenses

RUN sdkmanager "platform-tools" "platforms;android-${ANDROID_COMPILE_SDK}" "build-tools;${ANDROID_SDK_BUILD_TOOLS}"

WORKDIR /app

COPY . .


RUN chmod +x gradlew && \
    # ./gradlew assembleRelease
    ./gradlew assembleDebug
# app/build/outputs/apk/release/app-release.apk 생성(서명 필요)
# app/build/outputs/apk/debug/app-debug.apk 생성

# 컨테이너를 실행 상태로 유지하기 위해 로그 확인
ENTRYPOINT ["tail", "-f", "/dev/null"]
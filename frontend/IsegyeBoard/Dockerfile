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
    unzip cmdline-tools.zip && \
    rm cmdline-tools.zip && \
    mkdir -p cmdline-tools/latest && \
    mv cmdline-tools/* cmdline-tools/latest/ && \
    yes | sdkmanager --licenses

ENV PATH $PATH:/android-sdk/cmdline-tools/latest/bin

RUN sdkmanager "platform-tools;${ANDROID_SDK_PLATFORM_TOOLS}" "platforms;android-${ANDROID_COMPILE_SDK}" "build-tools;${ANDROID_SDK_BUILD_TOOLS}"

WORKDIR /app

COPY . .

# RUN ./gradlew assembleDebug
RUN chmod +x gradlew && \
    ./gradlew assembleRelease
# app/build/outputs/apk/release/app-release.apk 생성
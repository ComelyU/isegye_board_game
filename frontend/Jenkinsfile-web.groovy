def releasePort
def containerName

pipeline {
    agent any
    tools {
        nodejs "nodejs21.5.0"   
    }

    environment {
        webPath = 'frontend/Web/boardgame'

        gitBranch = 'develop-fe'
        gitCredential = 'demise1426-gitlab-sub'
        gitUrl = 'https://lab.ssafy.com/s10-final/S10P31A706.git'

        imageName = "demise1426/accio-isegye-web" // docker hub의 이미지 이름
        registryCredential = 'demise1426-docker' // docker hub access token

        releaseServerAccount = 'ubuntu' // ssh 연결 시 사용할 user
        releaseServerUri = 'k10a706.p.ssafy.io' // 서비스 url
        containerPort = '3000' // 컨테이너 포트
        bluePort = '3000' // blue포트
        greenPort = '3001' // green포트

        MATTERMOST_ENDPOINT = credentials('mattermost_endpoint')
        MATTERMOST_CHANNEL = credentials('mattermost_channel')
    }

    stages {
        stage('Check Changes') {
            steps {
                script {
                    // GitLab webhook payload contains information about the changes
                    def changes = currentBuild.rawBuild.changeSets.collect { changeLogSet ->
                        changeLogSet.collect { changeSet ->
                            changeSet.getAffectedFiles()
                        }
                    }.flatten()

                    // Check if changes include web boardgame directory
                    def webChanged = changes.any { it.path.startsWith(webPath) }

                    if (webChanged) {
                        echo 'Changes detected in frontend/Web/boardgame directory. Running the pipeline.'
                    } else {
                        echo 'No changes in frontend/Web/boardgame directory. Skipping the pipeline.'
                        currentBuild.result = 'ABORTED'
                        error 'No changes in frontend/Web/boardgame directory. Skipping the pipeline.'
                    }
                }
            }
        }

        stage('Git Clone') {
            steps {
                git branch: gitBranch,
                        credentialsId: gitCredential,
                        url: gitUrl
            }
        }

        stage('Node Build') {
            steps {
                dir('frontend/Web/boardgame') {
                    withCredentials([file(credentialsId: 'WEB_ENV', variable: 'WEB_ENV')]) {
                        withEnv(["MY_WEB_ENV=${WEB_ENV}"]) {
                            sh 'cp ${MY_WEB_ENV} .env'
                        }
                    }
                    sh 'npm install'
                    sh 'npm run build'
                }
            }
        }

        stage('Docker Image Build & DockerHub Push') {
            steps {
                dir('frontend/Web/boardgame') {
                    script {
                        docker.withRegistry('', registryCredential) {
                            sh "docker buildx create --use --name mybuilder"
                            sh "docker buildx build --platform linux/amd64,linux/arm64 -t $imageName:$BUILD_NUMBER -t $imageName:latest --push ."
                        }
                    }
                }
            }
        }

        stage('Blue/Green Port Check') { // 서비스 중단 전 기존 컨테이너 및 이미지 삭제
            steps {
                script {
                    // curl 명령어의 결과를 확인하여 포트 번호를 결정합니다.
                    def isBlueUp = sh(script: "curl -s --fail http://${releaseServerUri}:${bluePort}", returnStatus: true) == 0
                    if (isBlueUp) {
                        releasePort = greenPort
                        containerName = 'accio-isegye-web_g'
                    } else {
                        releasePort = bluePort
                        containerName = 'accio-isegye-web_b'
                    }
                    echo "isBlueUp : $isBlueUp, Port selected: $releasePort, container name: $containerName"
                }
            }
        }

        stage('DockerHub Pull') { // docker hub에서 이미지 pull
            steps {
                sshagent(credentials: ['SSH-ubuntu']) {
                    sh "ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri 'sudo docker pull $imageName:latest'"
                }
            }
        }

        stage('Service Start') { // pull된 이미지 이용하여 docker 컨테이너 실행
            steps {
                sshagent(credentials: ['SSH-ubuntu']) {
                    sh """
                    echo "port : ${releasePort}, container : ${containerName}"
                        ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri "sudo docker run -i -e TZ=Asia/Seoul --name ${containerName} -p ${releasePort}:${containerPort} -d ${imageName}:latest"
                    """
                }
            }
        }

        stage('Switch Nginx Port & Nginx reload') { //NginX Port 변경
            steps {
                sshagent(credentials: ['SSH-ubuntu']) {
                    sh """
                    ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri "echo 'set \\\$fe_service_url http://${releaseServerUri}:${releasePort};' | sudo tee /home/ubuntu/data/nginx/conf.d/fe-service-url.inc > /dev/null && sudo docker exec nginx nginx -s reload"
                    echo "Switch the reverse proxy direction of nginx to ${releasePort} 🔄"
                    """
                }
            }
        }

        stage('Service Check & Kill the Old Container') { // 연결 체크 & 예전 컨테이너 삭제
            steps {
                sshagent(credentials: ['SSH-ubuntu']) {
                    script {
                        def retry_count = 0
                        for (retry_count = 0; retry_count < 20; retry_count++) {
                            def isRunning = sh(script: "curl -s --fail http://${releaseServerUri}:${releasePort}/", returnStatus: true) == 0
                            if (isRunning) {
                                if(releasePort==bluePort){
                                    sh "ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri 'docker rm accio-isegye-web_g -f'"
                                }else{
                                    sh "ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri 'docker rm accio-isegye-web_b -f'"
                                }
                                echo "Killed the process on the opposite server."
                                sh "ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri 'docker image prune -f'"
                                break
                            } else {
                                if (retry_count == 19) {
                                    error("The server is not alive after 20 attempts. Exiting...")
                                }
                                echo "The server is not alive yet. Retry health check in 5 seconds..."
                                sleep 5
                            }
                        }
                    }
                }
            }
        }
    }

    post {
        success {
        	script {
                def Author_ID = sh(script: "git show -s --pretty=%an", returnStdout: true).trim()
                def Author_Name = sh(script: "git show -s --pretty=%ae", returnStdout: true).trim()
                mattermostSend (
                    color: 'good', 
                    message: "Fe-Web Build Success: ${env.JOB_NAME} #${env.BUILD_NUMBER} by ${Author_ID}(${Author_Name})\n(<${env.BUILD_URL}|Details>)", 
                    endpoint: MATTERMOST_ENDPOINT, 
                    channel: MATTERMOST_CHANNEL
                )
            }
        }
        failure {
        	script {
                def Author_ID = sh(script: "git show -s --pretty=%an", returnStdout: true).trim()
                def Author_Name = sh(script: "git show -s --pretty=%ae", returnStdout: true).trim()
                mattermostSend (
                    color: 'danger', 
                    message: "Fe-Web Build Failure: ${env.JOB_NAME} #${env.BUILD_NUMBER} by ${Author_ID}(${Author_Name})\n(<${env.BUILD_URL}|Details>)", 
                    endpoint: MATTERMOST_ENDPOINT, 
                    channel: MATTERMOST_CHANNEL
                )
            }
        }
    }
}
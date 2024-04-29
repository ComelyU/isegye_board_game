def s3url

pipeline {
    agent any

    environment {
        targetPath = 'frontend/IsegyeBoard'

        gitBranch = 'develop-fe'
        gitCredential = 'demise1426-gitlab-sub'
        gitUrl = 'https://lab.ssafy.com/s10-final/S10P31A706.git'

        imageName = "demise1426/accio-isegye-android-customer" // docker hub의 이미지 이름
        registryCredential = 'demise1426-docker' // docker hub access token
        
        apkFileContainerPath = '/app/build/outputs/apk/debug/app-debug.apk'
        apkFileLocalPath = '/home/ubuntu/apk/customer/'
        apkS3Path = 'apk/customer/'
        s3BucketName = 'accio-isegye'
        awsRegion = 'ap-northeast-2'

        releaseServerAccount = 'ubuntu' // ssh 연결 시 사용할 user
        releaseServerUri = 'k10a706.p.ssafy.io' // 서비스 url
        containerName = 'accio-isegye-android-customer'

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

                    // Check if changes include frontend customer app directory
                    def targetChanged = changes.any { it.path.startsWith(targetPath) }

                    if (targetChanged) {
                        echo 'Changes detected in frontend/IsegyeBoard directory. Running the pipeline.'
                    } else {
                        echo 'No changes in frontend/IsegyeBoard directory. Skipping the pipeline.'
                        currentBuild.result = 'ABORTED'
                        error 'No changes in frontend/IsegyeBoard directory. Skipping the pipeline.'
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

        stage('Docker Image Build & DockerHub Push') {
            steps {
                dir('frontend/IsegyeBoard') {
                    script {
                        docker.withRegistry('', registryCredential) {
                            sh "docker buildx create --use --name mybuilder"
                            sh "docker buildx build --platform linux/amd64 -t $imageName:$BUILD_NUMBER --push ."
                            sh "docker buildx build --platform linux/amd64 -t $imageName:latest --push ."
                        }
                    }
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

        stage('Docker Run & File Copy') {
            steps {
                sshagent(credentials: ['SSH-ubuntu']) {
                    sh """
                        ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri "sudo docker run --name ${containerName} -d ${imageName}:latest"
                        ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri "rm -f ${apkFileLocalPath}app-debug.apk"
                        ssh -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri "sudo docker cp ${containerName}:${apkFileContainerPath} ${apkFileLocalPath}"
                        scp -o StrictHostKeyChecking=no $releaseServerAccount@$releaseServerUri:${apkFileLocalPath}app-debug.apk .
                    """
                }
            }
        }
        
        stage('S3 Upload') {
            steps {
                withCredentials([aws(credentialsId: 'AWS-IAM', accessKeyVariable: 'AWS_ACCESS_KEY_ID', secretKeyVariable : 'AWS_SECRET_ACCESS_KEY ')]) {
                    sh "aws s3 cp ./app-debug.apk s3://${s3BucketName}/${apkS3Path}${BUILD_NUMBER}/app-debug.apk"
                    script {
                        s3url = "https://${s3BucketName}.s3.${awsRegion}.amazonaws.com/${apkS3Path}${BUILD_NUMBER}/app-debug.apk"
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
                    message: "Fe-Customer App Build Success: ${env.JOB_NAME} #${env.BUILD_NUMBER} by ${Author_ID}(${Author_Name})\nAPK FILE S3 LINK:${s3url}\n(<${env.BUILD_URL}|Details>)", 
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
                    message: "Fe-Customer App Build Failure: ${env.JOB_NAME} #${env.BUILD_NUMBER} by ${Author_ID}(${Author_Name})\n(<${env.BUILD_URL}|Details>)", 
                    endpoint: MATTERMOST_ENDPOINT, 
                    channel: MATTERMOST_CHANNEL
                )
            }
        }
    }
}
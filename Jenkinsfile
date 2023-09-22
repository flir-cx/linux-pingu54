pipeline {
  agent {
    label 'amd64'
  }

  options {
    buildDiscarder(
      logRotator(
        daysToKeepStr: '90',
        numToKeepStr: '100'
      )
    )
  }

  stages {
    stage ('Run Checkpatch script') {
      steps {
        // For some reason the base branch remote is called 'upstream'
        script {
          if (env.CHANGE_TARGET) {
            sh 'scripts/checkpatch.pl --git upstream/${CHANGE_TARGET}..HEAD'
          }
          else {
            sh 'scripts/checkpatch.pl --git HEAD'
          }
        }
      }
    }
  }
  post {
    always {
      cleanWs()
    }
  }
}

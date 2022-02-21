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
        sh 'scripts/checkpatch.pl --git upstream/FLIR_lf-5.10.y..HEAD'
      }
    }
  }
  post {
    always {
      cleanWs()
    }
  }
}

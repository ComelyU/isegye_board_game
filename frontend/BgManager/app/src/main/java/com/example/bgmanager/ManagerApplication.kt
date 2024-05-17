package com.example.bgmanager

import android.app.Application
import dagger.hilt.android.HiltAndroidApp

@HiltAndroidApp
class ManagerApplication : Application() {
    companion object {
        private lateinit var applicaton: ManagerApplication
        fun getInstance(): ManagerApplication = applicaton
    }

    override fun onCreate() {
        super.onCreate()
        applicaton = this
    }
}
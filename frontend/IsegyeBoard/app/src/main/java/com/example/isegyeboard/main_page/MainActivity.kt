package com.example.isegyeboard.main_page

import android.os.Bundle
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.navigation.findNavController
import com.example.isegyeboard.MainNavDirections
import com.example.isegyeboard.R

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_main)

        val logoButton = findViewById<ConstraintLayout>(R.id.logoButton)
        logoButton.setOnClickListener{
            val action = MainNavDirections.actionGlobalHome()
            findNavController(R.id.mainFragmentView).navigate(action)
        }
    }
}
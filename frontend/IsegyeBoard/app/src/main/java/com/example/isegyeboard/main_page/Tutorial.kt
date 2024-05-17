package com.example.isegyeboard.main_page

import android.os.Bundle
import android.view.View
import android.widget.ImageView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import com.example.isegyeboard.R

class Tutorial : AppCompatActivity() {

    private lateinit var imageView: ImageView
    private lateinit var imageView2: ImageView
    private lateinit var imageView3: ImageView
    private lateinit var imageView4: ImageView
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_tutorial)

        imageView = findViewById(R.id.tutorialImage)
        imageView2 = findViewById(R.id.tutorialImage2)
        imageView3 = findViewById(R.id.tutorialImage3)
        imageView4 = findViewById(R.id.tutorialImage4)

        imageView.setOnClickListener{
            imageView.visibility = View.GONE
            imageView2.visibility = View.VISIBLE
        }

        imageView2.setOnClickListener{
            imageView2.visibility = View.GONE
            imageView3.visibility = View.VISIBLE
        }

        imageView3.setOnClickListener{
            imageView3.visibility = View.GONE
            imageView4.visibility = View.VISIBLE
        }

        imageView4.setOnClickListener{
            finish()
        }
    }
}
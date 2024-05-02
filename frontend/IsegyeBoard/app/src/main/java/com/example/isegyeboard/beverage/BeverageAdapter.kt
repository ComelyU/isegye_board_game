package com.example.isegyeboard.beverage

import android.content.Context
import android.graphics.Color
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.findNavController
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.isegyeboard.R
import com.example.isegyeboard.game_list.GameAdapter
import com.example.isegyeboard.game_list.GameClass
import kotlin.math.ceil


class BeverageAdapter(private val context: Context, private var dataList: List<BeverageClass>) :
    RecyclerView.Adapter<BeverageAdapter.BeverageViewHolder>() {

    inner class BeverageViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.menuName)
        val priceTextView: TextView = itemView.findViewById(R.id.menuPrice)


        init {
            itemView.setOnClickListener(this)
        }

        override fun onClick(v: View?) {
            val menu = dataList[adapterPosition]
            if (menu.isAvailable == true) {
                val bundle = Bundle().apply {
                    putString("gameId", menu.id.toString())

                }

            } else {
                Log.e("List", "e")
            }
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): BeverageAdapter.BeverageViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_menu_item, parent, false)
        return BeverageViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: BeverageAdapter.BeverageViewHolder, position: Int) {
        val gameItem = dataList[position]

//        Glide.with(context)
//            .load(gameItem.thumbnailUrl)
//            .placeholder(R.drawable.ipad) // 로딩이미지
//            .error(R.drawable.chess_black) //실패이미지
//            .into(holder.imageView)
    }
    override fun getItemCount() = dataList.size

    fun updateData(newMenuList: List<BeverageClass>) {
        dataList = newMenuList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }
}

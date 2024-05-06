package com.example.isegyeboard.beverage

import android.content.Context
import android.graphics.Color
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.isegyeboard.R
import com.example.isegyeboard.beverage.cart.CartClass
import com.example.isegyeboard.beverage.cart.CartViewModel
import com.example.isegyeboard.game_list.GameAdapter
import kotlin.math.ceil

class BeverageAdapter(
    private val context: Context,
    private var dataList: List<BeverageClass>,
//    private val beverageViewModel: BeverageViewModel,
//    private val cartViewModel: CartViewModel
) : RecyclerView.Adapter<BeverageAdapter.BeverageViewHolder>() {

    inner class BeverageViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.menuName)
        val priceTextView: TextView = itemView.findViewById(R.id.menuPrice)
        val imageView: ImageView = itemView.findViewById(R.id.menuImage)


        init {
            itemView.setOnClickListener(this)
        }

        override fun onClick(v: View?) {
            val menu = dataList[adapterPosition]
            if (menu.isAvailable == 1) {
                val cartItem = CartClass(menu.menuName, menu.menuPrice, 1)

            } else {
                Log.e("BeverageList", "e")
            }
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): BeverageAdapter.BeverageViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_menu_item, parent, false)
        return BeverageViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: BeverageAdapter.BeverageViewHolder, position: Int) {
        val menuItem = dataList[position]

        println("뷰홀더 ${menuItem}")

        if (menuItem.isAvailable == 0) {
            holder.itemView.setBackgroundColor(Color.LTGRAY)
            holder.nameTextView.setTextColor(Color.GRAY)
            holder.priceTextView.setTextColor(Color.GRAY)
        } else {
            holder.itemView.setBackgroundColor(Color.WHITE)
        }
        holder.nameTextView.text = menuItem.menuName
        holder.priceTextView.text = "${menuItem.menuPrice} 원"
        Glide.with(context)
            .load(menuItem.menuImgUrl)
            .placeholder(R.drawable.ipad) // 로딩이미지
            .error(R.drawable.chess_black) //실패이미지
            .into(holder.imageView)
    }
    override fun getItemCount() = dataList.size

    fun updateData(newMenuList: List<BeverageClass>) {
        dataList = newMenuList
    }
}

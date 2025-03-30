use bevy::prelude::*;
use bevy_hanabi::prelude::*;
pub fn make_explosion_effect() -> EffectAsset {
    let mut gradient = Gradient::new();
    gradient.add_key(0.0, Vec4::new(0.5, 0.5, 1.0, 1.0));
    gradient.add_key(1.0, Vec4::new(0.5, 0.5, 1.0, 0.0));

    let writer = ExprWriter::new();

    let age = writer.lit(0.).expr();
    let init_age = SetAttributeModifier::new(Attribute::AGE, age);

    let lifetime = writer.lit(2.0).expr();
    let init_lifetime = SetAttributeModifier::new(Attribute::LIFETIME, lifetime);

    let init_pos = SetPositionCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        radius: writer.lit(0.05).expr(),
        dimension: ShapeDimension::Surface,
    };

    let speed = writer.add_property("speed", Value::Scalar(ScalarValue::Float(100.0)));
    let speed = writer.prop(speed);

    let init_vel = SetVelocityCircleModifier {
        center: writer.lit(Vec3::ZERO).expr(),
        axis: writer.lit(Vec3::Z).expr(),
        speed: (writer.rand(ValueType::Scalar(ScalarType::Float))
            * (writer.lit(3.0)
                - writer.lit(2.0) * writer.rand(ValueType::Scalar(ScalarType::Float)))
            * speed)
            .expr(),
    };

    let drag = writer.add_property("drag", Value::Scalar(ScalarValue::Float(4.0)));
    let drag = writer.prop(drag).expr();

    let update_drag = LinearDragModifier::new(drag);

    let module = writer.finish();

    let spawner = Spawner::once(100.0.into(), true);
    EffectAsset::new(vec![2048], spawner, module)
        .with_name("2d_default")
        .init(init_pos)
        .init(init_vel)
        .init(init_age)
        .init(init_lifetime)
        .update(update_drag)
        .render(SizeOverLifetimeModifier {
            gradient: Gradient::constant(Vec2::splat(2.0)),
            screen_space_size: false,
        })
        .render(OrientModifier {
            mode: OrientMode::AlongVelocity,
            rotation: None,
        })
        .with_simulation_space(SimulationSpace::Local)
        .build()
}
